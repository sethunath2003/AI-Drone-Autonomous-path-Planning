/*
 * ITERATION 6: Autonomous Flight + Landing Mode
 * - FEATURES:
 * 1. Normal: Flies to waypoints (PD stabilized).
 * 2. Land Mode: Descends vertically and cuts motors on ground.
 * 3. Toggle: Press 'L' to switch between Flying and Landing.
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
#include <webots/motor.h>
#include <webots/keyboard.h> // Added for input

// === UTILITIES ===
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define PI 3.14159265358979323846

// === TUNING CONSTANTS ===
const double k_vertical_thrust = 68.5;  
const double k_vertical_offset = 0.6; 
const double k_vertical_p = 3.0;      
const double k_roll_p = 50.0;           
const double k_pitch_p = 30.0;
const double k_yaw_p = 2.0;   
const double k_yaw_d = 5.0;   // Damping to stop spin

const double obstacle_threshold = 1.5; 
const double waypoint_tolerance = 1.0; 

typedef struct { double x, y, z; } Waypoint;
#define MAX_WAYPOINTS 10
Waypoint path[MAX_WAYPOINTS];
int total_waypoints = 0;
int current_wp_index = 0;

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // === USER VARIABLES ===
  bool land_mode = false;  // Set this to TRUE to start on the ground, or toggle with 'L'

  // === WAYPOINTS ===
  path[0].x = 15.0;  path[0].y = 35.0; path[0].z = 22.0;
  path[1].x = 0.0;   path[1].y = 0.0;   path[1].z = 2.0;
  total_waypoints = 2; 

  // === DEVICES ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);
  int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  
  wb_keyboard_enable(timestep); // Enable keyboard

  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  printf("Ready. Press 'L' to toggle Landing Mode.\n");

  while (wb_robot_step(timestep) != -1) {
    
    // 1. INPUTS
    int key = wb_keyboard_get_key();
    while (key > 0) {
        if (key == 'L') {
            land_mode = !land_mode;
            if (land_mode) printf(">> COMMAND: LANDING!\n");
            else printf(">> COMMAND: RESUMING FLIGHT!\n");
        }
        key = wb_keyboard_get_key();
    }

    // 2. SENSORS
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2];

    const double *gps_vals = wb_gps_get_values(gps);
    double cur_x = gps_vals[0];
    double cur_y = gps_vals[1];
    double cur_alt = gps_vals[2];
    
    const double *gyro_vals = wb_gyro_get_values(gyro);
    double yaw_velocity = gyro_vals[2];

    // 3. FLIGHT LOGIC
    double target_z = 2.0; // Default flight altitude
    double pitch_cmd = 0.0;
    double roll_cmd = 0.0;
    double yaw_cmd = 0.0;

    if (land_mode) {
        // === LANDING LOGIC ===
        target_z = 0.0; // Aim for ground
        
        // Stop all horizontal movement
        pitch_cmd = 0.0;
        roll_cmd = 0.0;
        yaw_cmd = 0.0; 

        // ENGINE CUTOFF: If we are on the ground, stop motors
        if (cur_alt < 0.15) {
            wb_motor_set_velocity(front_left_motor, 0);
            wb_motor_set_velocity(front_right_motor, 0);
            wb_motor_set_velocity(rear_left_motor, 0);
            wb_motor_set_velocity(rear_right_motor, 0);
            continue; // Skip PID and restart loop
        }
    } 
    else {
        // === WAYPOINT LOGIC ===
        double target_x = path[current_wp_index].x;
        double target_y = path[current_wp_index].y;
        target_z = path[current_wp_index].z;

        double dx = target_x - cur_x;
        double dy = target_y - cur_y;
        double dist = sqrt(dx*dx + dy*dy);

        // Check Arrival
        if (dist < waypoint_tolerance && fabs(cur_alt - target_z) < 0.5) {
            printf(">> WP %d Reached.\n", current_wp_index);
            current_wp_index++;
            if (current_wp_index >= total_waypoints) {
                printf(">> Destination Reached. Hovering.\n");
                current_wp_index = total_waypoints - 1; 
            }
        }

        // Navigation
        double angle_to_target = atan2(dy, dx); 
        double heading_error = angle_to_target - yaw;
        while (heading_error > PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;

        // Obstacle Check
        const float *lidar_values = wb_lidar_get_range_image(lidar);
        double center_dist = lidar_values[lidar_width / 2];
        bool obstacle = (center_dist < obstacle_threshold && center_dist > 0.1);

        if (obstacle) {
            target_z += 0.08; // Fly over obstacle
        } else {
            // PD Yaw Control
            yaw_cmd = (k_yaw_p * heading_error) - (k_yaw_d * yaw_velocity);
            yaw_cmd = CLAMP(yaw_cmd, -1.0, 1.0); 

            // Fly forward if aligned
            if (fabs(heading_error) < 0.8) { 
                 double speed_factor = (0.8 - fabs(heading_error)) / 0.8;
                 pitch_cmd = -2.0 * speed_factor; 
                 if (pitch_cmd > -0.5) pitch_cmd = -0.5;
            }
        }
    }

    // 4. MOTOR MIXING & PID
    double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + gyro_vals[0] + roll_cmd;
    double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + gyro_vals[1] + pitch_cmd;
    double yaw_input = yaw_cmd;
    double alt_input = k_vertical_p * pow(CLAMP(target_z - cur_alt + k_vertical_offset, -1.0, 1.0), 3.0);

    double fl = k_vertical_thrust + alt_input - roll_input + pitch_input - yaw_input;
    double fr = k_vertical_thrust + alt_input + roll_input + pitch_input + yaw_input;
    double rl = k_vertical_thrust + alt_input - roll_input - pitch_input + yaw_input;
    double rr = k_vertical_thrust + alt_input + roll_input - pitch_input - yaw_input;

    wb_motor_set_velocity(front_left_motor, fl);
    wb_motor_set_velocity(front_right_motor, -fr);
    wb_motor_set_velocity(rear_left_motor, -rl);
    wb_motor_set_velocity(rear_right_motor, rr);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}