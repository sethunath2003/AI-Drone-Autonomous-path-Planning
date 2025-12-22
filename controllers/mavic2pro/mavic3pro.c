/*
 * AI Drone Controller with Lidar-based Obstacle Avoidance
 * - Stabilizes drone using IMU, GPS, gyro
 * - Maintains altitude automatically
 * - Uses Lidar to detect obstacles ahead
 * - Moves forward if no obstacle, stops otherwise
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/led.h>
#include <webots/motor.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// === Constants (tuned empirically for the Webots drone model) ===
const double k_vertical_thrust = 68.5;   // Base lift thrust
const double k_vertical_offset = 0.6;    // Vertical offset for hovering
const double k_vertical_p = 3.0;         // Vertical PID P gain
const double k_roll_p = 50.0;            // Roll stabilization P gain
const double k_pitch_p = 30.0;           // Pitch stabilization P gain

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // === Devices ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");

  // Motors
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // === Lidar setup ===
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);
  int lidar_width = wb_lidar_get_horizontal_resolution(lidar);

  printf("AI Drone starting with Lidar-based control...\n");

  double target_altitude = 1.0;  // Hover at 1m

  while (wb_robot_step(timestep) != -1) {
    double time = wb_robot_get_time();

    // === Sensor readings ===
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    double altitude = wb_gps_get_values(gps)[2];
    const double *gyro_values = wb_gyro_get_values(gyro);
    double roll_velocity = gyro_values[0];
    double pitch_velocity = gyro_values[1];

    // === LED blinking ===
    bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // === Lidar processing ===
    const float *lidar_values = wb_lidar_get_range_image(lidar);
    int center_index = lidar_width / 2;
    double forward_distance = lidar_values[center_index];

    // === Control logic ===
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;  // Forward/backward movement
    double yaw_disturbance = 0.0;

    // Obstacle avoidance:
    if (forward_distance > 1.0) {
      // No obstacle in 1m range → move forward
      pitch_disturbance = -2.0;
      printf("Moving forward (distance=%.2f)\n", forward_distance);
    } else {
      // Obstacle close → hover
      pitch_disturbance = 0.0;
      printf("Stopping: obstacle ahead (distance=%.2f)\n", forward_distance);
    }

    // === PID Stabilization ===
    double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    double yaw_input = yaw_disturbance;
    double clamped_diff_alt = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vertical_input = k_vertical_p * pow(clamped_diff_alt, 3.0);

    // === Motor mixing ===
    double fl = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    double fr = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    double rl = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    double rr = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;

    wb_motor_set_velocity(front_left_motor, fl);
    wb_motor_set_velocity(front_right_motor, -fr);
    wb_motor_set_velocity(rear_left_motor, -rl);
    wb_motor_set_velocity(rear_right_motor, rr);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
