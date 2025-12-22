/*
 * ITERATION 1.6: Manual Control + Smart Obstacle Negotiation
 * - Fixed: Unused variable warning removed.
 * - Added: On-screen control instructions.
 * - Tuned: Controls are "snappier" (higher tilt angles).
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
#include <webots/keyboard.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// === Tuning Constants ===
const double k_vertical_thrust = 68.5;  
const double k_vertical_offset = 0.6; 
const double k_vertical_p = 3.0;      
const double k_roll_p = 50.0;           
const double k_pitch_p = 30.0;          
const double obstacle_threshold = 1.5; // Distance to trigger avoidance

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // === Enable Devices ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  wb_keyboard_enable(timestep);

  // === Lidar setup ===
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);
  int lidar_width = wb_lidar_get_horizontal_resolution(lidar);

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

  // === PRINT CONTROLS TO CONSOLE ===
  printf("\n");
  printf("=========================================\n");
  printf("   DRONE FLIGHT CONTROLS (Click 3D View) \n");
  printf("=========================================\n");
  printf(" [ Up Arrow ]    : Fly Forward\n");
  printf(" [ Down Arrow ]  : Fly Backward\n");
  printf(" [ Left Arrow ]  : Strafe Left\n");
  printf(" [ Right Arrow ] : Strafe Right\n");
  printf(" [ W ]           : Ascend (Go Up)\n");
  printf(" [ S ]           : Descend (Go Down)\n");
  printf(" [ A ]           : Rotate Left (Yaw)\n");
  printf(" [ D ]           : Rotate Right (Yaw)\n");
  printf("=========================================\n\n");

  double target_altitude = 1.0; 

  while (wb_robot_step(timestep) != -1) {
    // Note: 'time' variable removed to fix compiler warning

    // === Read Sensors ===
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    double altitude = wb_gps_get_values(gps)[2];
    const double *gyro_values = wb_gyro_get_values(gyro);
    double roll_velocity = gyro_values[0];
    double pitch_velocity = gyro_values[1];

    // === Lidar Analysis ===
    const float *lidar_values = wb_lidar_get_range_image(lidar);
    
    int left_sector = 0;
    int center_sector = 0;
    int right_sector = 0;
    int total_blocked = 0;

    for (int i = 0; i < lidar_width; i++) {
        // Filter out infinity or bad readings
        if (lidar_values[i] < obstacle_threshold && lidar_values[i] > 0.1) {
            total_blocked++;
            if (i < lidar_width / 3) left_sector++;
            else if (i < (2 * lidar_width) / 3) center_sector++;
            else right_sector++;
        }
    }

    double center_distance = lidar_values[lidar_width / 2];
    bool obstacle_ahead = (center_distance < obstacle_threshold && center_distance > 0.1);

    // === Manual Input ===
    int key = wb_keyboard_get_key();
    
    double pitch_cmd = 0.0;
    double roll_cmd = 0.0;
    double yaw_cmd = 0.0;

    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:    
            pitch_cmd = -2.5; // Increased for snappier response
            break; 
        case WB_KEYBOARD_DOWN:  
            pitch_cmd = 2.5;  
            break; 
        case WB_KEYBOARD_RIGHT: 
            roll_cmd = -1.5;  // Increased for snappier response
            break; 
        case WB_KEYBOARD_LEFT:  
            roll_cmd = 1.5;   
            break; 
        case 'W': 
            target_altitude += 0.08; // Faster ascent
            break;
        case 'S': 
            target_altitude -= 0.08; 
            break;
        case 'D': 
            yaw_cmd = -2.0; // Faster rotation
            break;
        case 'A': 
            yaw_cmd = 2.0;  
            break;
      }
      key = wb_keyboard_get_key();
    }

    // === SMART AVOIDANCE LOGIC ===
    if (obstacle_ahead && pitch_cmd < -0.1) {
        
        double blocked_ratio = (double)total_blocked / lidar_width;

        // WIDE OBSTACLE -> FLY UP
        if (blocked_ratio > 0.4) { // Lowered threshold slightly for safety
            // printf("Avoiding Wall: Going UP\n"); // Uncomment to debug
            pitch_cmd = 0.0;          
            target_altitude += 0.1;  
        } 
        // NARROW OBSTACLE -> STRAFE
        else {
             // printf("Avoiding Pole: Strafing\n"); // Uncomment to debug
             pitch_cmd = 0.0; 
             
             if (left_sector > right_sector) {
                 roll_cmd = -2.0; // Slide Right
             } else {
                 roll_cmd = 2.0;  // Slide Left
             }
        }
    }

    // === PID Control ===
    double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_cmd;
    double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_cmd;
    
    double clamped_diff_alt = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vertical_input = k_vertical_p * pow(clamped_diff_alt, 3.0);

    // Motor Mixing
    double fl = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_cmd;
    double fr = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_cmd;
    double rl = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_cmd;
    double rr = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_cmd;

    // Correct: Clamp motor values to ensure no reversal (negative thrust) and no overspeed
    wb_motor_set_velocity(front_left_motor, CLAMP(fl, 0.0, 576.0));
    wb_motor_set_velocity(front_right_motor, -CLAMP(fr, 0.0, 576.0));
    wb_motor_set_velocity(rear_left_motor, -CLAMP(rl, 0.0, 576.0));
    wb_motor_set_velocity(rear_right_motor, CLAMP(rr, 0.0, 576.0));
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}