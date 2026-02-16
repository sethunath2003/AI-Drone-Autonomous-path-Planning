/*
 * ITERATION 10: Fixed Lidar Integration & Stable Flight
 * - Based on the stable "mavic2pro_v3" controller.
 * - Adds PROPER Lidar support with "lidar" device.
 * - Simple obstacle avoidance (Stops and Yaws away).
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
#include <webots/keyboard.h>

// === UTILITIES ===
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define PI 3.14159265358979323846

// === TUNING CONSTANTS (From v3 - Validated) ===
const double k_vertical_thrust = 68.5;  
const double k_vertical_offset = 0.6; 
const double k_vertical_p = 3.0;      
const double k_roll_p = 50.0;           
const double k_pitch_p = 30.0;
const double k_yaw_p = 2.0;   
const double k_yaw_d = 5.0; 

const double obstacle_threshold = 2.0; // Distance to stop at
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
  bool land_mode = false;

  // === WAYPOINTS ===
  // v3 was visiting (-29, 0, 1) then (0, 0, 2).
  // TEST MODE: Fly DIRECTLY at the Tesla to test Lidar.
  // We set Z = 1.0m (approx 3 feet) to hit the car "center-mass".
  // If Lidar works, it should STOP before hitting.
  path[0].x = -41.5; path[0].y = 4.34; path[0].z = 1.0; 
  total_waypoints = 1; 

  // === DEVICES ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  
  // === LIDAR SETUP ===
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  if (lidar > 0) {
      wb_lidar_enable(lidar, timestep);
      wb_lidar_enable_point_cloud(lidar); // Optional, for visualization
  } else {
      printf("Warning: 'lidar' device not found on robot. Obstacle avoidance disabled.\n");
  }

  wb_keyboard_enable(timestep);

  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  printf("Mavic 2 Pro v5 Initialized.\n");
  printf("Press 'L' to toggle Landing Mode.\n");

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
    double cur_alt = gps_vals[2]; // Z-up world assumption from v3
    
    const double *gyro_vals = wb_gyro_get_values(gyro);
    double yaw_velocity = gyro_vals[2];

    // 3. FLIGHT LOGIC
    double target_z = 2.0; 
    double pitch_cmd = 0.0;
    double roll_cmd = 0.0;
    double yaw_cmd = 0.0;

    if (land_mode) {
        target_z = 0.0; 
        // Stop horizontal movement
        pitch_cmd = 0.0;
        roll_cmd = 0.0;
        yaw_cmd = 0.0; 

        if (cur_alt < 0.15) {
            // Cut motors
            for (int m=0; m<4; m++) wb_motor_set_velocity(motors[m], 0.0);
            continue; 
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

        // Navigation (Angle to target)
        double angle_to_target = atan2(dy, dx); 
        double heading_error = angle_to_target - yaw;
        while (heading_error > PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;

        // === OBSTACLE CHECK ===
        bool obstacle_detected = false;
        double obstacle_dist = 100.0;

        if (lidar > 0) {
            const float *lidar_values = wb_lidar_get_range_image(lidar);
            if (lidar_values) {
                int w = wb_lidar_get_horizontal_resolution(lidar);
                // Check center 20% of the image
                int mid = w / 2;
                double center_avg = 0;
                int count = 0;
                for (int i = mid - 20; i < mid + 20; i++) {
                    if (i >=0 && i < w && !isinf(lidar_values[i])) {
                         center_avg += lidar_values[i];
                         count++;
                    }
                }
                if (count > 0) {
                    obstacle_dist = center_avg / count;
                    // DEBUG: Print Lidar stats every 50 steps
                    static int debug_counter = 0;
                    if (debug_counter++ % 50 == 0) {
                        printf(">> LIDAR DEBUG: Center Dist: %.2f (Threshold: %.2f)\n", obstacle_dist, obstacle_threshold);
                    }
                    if (obstacle_dist < obstacle_threshold && obstacle_dist > 0.1) {
                         obstacle_detected = true;
                    }
                }
            }
        }

        // === CONTROL ===
        if (obstacle_detected) {
            // Emergency Stop / Avoid
            if (obstacle_dist < 1.0) {
                 // Too close! Hover and Turn
                 pitch_cmd = 0.0; 
                 yaw_cmd = 0.5; // Turn away
                 printf(">> OBSTACLE (%.2fm)! Stopping.\n", obstacle_dist);
            } else {
                 // Slow down
                 pitch_cmd = -0.2; 
                 yaw_cmd = 0.5; // Turn slowly
                 printf(">> OBSTACLE (%.2fm)! Slowing.\n", obstacle_dist);
            }
        } 
        else {
            // Normal Navigation
            yaw_cmd = (k_yaw_p * heading_error) - (k_yaw_d * yaw_velocity);
            yaw_cmd = CLAMP(yaw_cmd, -1.0, 1.0); 

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
