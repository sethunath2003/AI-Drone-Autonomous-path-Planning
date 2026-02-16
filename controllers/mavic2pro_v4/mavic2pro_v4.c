/*
 * ITERATION 10: Final Flight + Self-Filtering Lidar
 * - PROBLEM SOLVED: Lidar seeing propellers caused drone to freeze.
 * - FIX: Added "Min Distance Filter" (Ignore < 0.5m).
 * - FEATURES: Auto-Calibration, Y-Up Altitude, Obstacle Avoidance.
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

// === TUNING CONSTANTS ===
const double k_vertical_thrust = 68.5; 
const double k_vertical_offset = 0.6;
const double k_vertical_p = 3.0;
const double k_roll_p = 50.0;
const double k_pitch_p = 30.0;
const double k_yaw_p = 2.0;
const double k_yaw_d = 5.0;

// === ODA TUNING ===
const double obstacle_threshold = 5.0; // Stop if object is within 5m
const double min_safe_dist = 0.5;      // IGNORE objects closer than 0.5m (Propellers)
const double avoid_speed = 1.0;        
const double waypoint_tolerance = 1.5;

// === STRUCTS ===
typedef struct { double x, z; } Waypoint; 
#define MAX_WAYPOINTS 10
Waypoint path[MAX_WAYPOINTS];
int total_waypoints = 0;
int current_wp_index = 0;

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // === USER VARIABLES ===
  bool land_mode = false;
  int avoid_counter = 0; 
  double ground_level = 0.0; 
  bool is_calibrated = false;
  int calibration_timer = 0;

  // === WAYPOINTS ===
  // Flying to X=0, Z=0 (Adjust these to match your House location)
  path[0].x = 0.0;    path[0].z = 0.0; 
  total_waypoints = 1;

  // === DEVICES ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  
  // === LIDAR SETUP ===
  // Ensure your Scene Tree device is named "obstacle_lidar" or "lidar"
  WbDeviceTag lidar = wb_robot_get_device("obstacle_lidar"); 
  if (lidar == 0) lidar = wb_robot_get_device("lidar"); // Fallback check

  if (lidar != 0) {
      wb_lidar_enable(lidar, timestep);
      printf(">> Lidar connected. Enabling Self-Filter...\n");
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

  printf(">> SYSTEM BOOT. Calibrating Ground Level (Wait 1s)...\n");

  while (wb_robot_step(timestep) != -1) {
    
    // === READ SENSORS ===
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2]; 

    const double *gps_vals = wb_gps_get_values(gps);
    double cur_x = gps_vals[0]; 
    double cur_y = gps_vals[1]; // Altitude
    double cur_z = gps_vals[2]; 
    
    const double *gyro_vals = wb_gyro_get_values(gyro);
    double yaw_velocity = gyro_vals[2];

    // === CALIBRATION PHASE ===
    if (!is_calibrated) {
        calibration_timer++;
        for(int i=0; i<4; i++) wb_motor_set_velocity(motors[i], 0);

        if (calibration_timer > 50) { 
            ground_level = cur_y; 
            is_calibrated = true;
            printf(">> CALIBRATED. Ground: %.2fm.\n", ground_level);
        }
        continue; 
    }

    double rel_alt = cur_y - ground_level;

    // === INPUTS ===
    int key = wb_keyboard_get_key();
    while (key > 0) {
        if (key == 'L') land_mode = !land_mode;
        key = wb_keyboard_get_key();
    }

    // === FLIGHT LOGIC ===
    double target_rel_alt = 1.0; //chnaged from 2 to 1
    double pitch_cmd = 0.0;
    double roll_cmd = 0.0;
    double yaw_cmd = 0.0;

    if (land_mode) {
        target_rel_alt = 0.0;
        if (rel_alt < 0.2) {
            for(int i=0; i<4; i++) wb_motor_set_velocity(motors[i], 0);
            continue; 
        }
    } 
    else {
        // --- NAVIGATION ---
        double target_x = path[current_wp_index].x;
        double target_z = path[current_wp_index].z; 

        double dx = target_x - cur_x;
        double dz = target_z - cur_z; 
        double dist_2d = sqrt(dx*dx + dz*dz);
        
        // Navigation Logic
        double angle_to_target = atan2(dz, dx); 
        double heading_error = angle_to_target - yaw;
        while (heading_error > PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;

        yaw_cmd = (k_yaw_p * heading_error) - (k_yaw_d * yaw_velocity);
        yaw_cmd = CLAMP(yaw_cmd, -1.0, 1.0); 

        if (fabs(heading_error) < 1.0) { 
             double speed_factor = (1.0 - fabs(heading_error));
             pitch_cmd = -2.0 * speed_factor; 
             pitch_cmd = CLAMP(pitch_cmd, -2.0, 0.0);
        }

        // --- OBSTACLE AVOIDANCE (THE FIX) ---
        if (lidar != 0) {
            int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
            const float *lidar_values = wb_lidar_get_range_image(lidar);
            
            if (lidar_values) {
                int center_idx = lidar_width * 0.5;  
                double d_center = lidar_values[center_idx];

                if(isinf(d_center)) d_center = 100.0;

                // DEBUG PRINT: See what the lidar actually sees
                avoid_counter++;
                if(avoid_counter % 50 == 0) {
                    // printf(">> DEBUG: Lidar dist: %.2f\n", d_center);
                }

                // FILTER: Only react if Distance > 0.5m (Ignore self) AND Distance < Threshold
                bool real_obstacle = (d_center > min_safe_dist) && (d_center < obstacle_threshold);

                if (rel_alt > 0.5 && real_obstacle) {
                    if (avoid_counter % 20 == 0) printf(">> WARNING: OBSTACLE AT %.2fm! BRAKING.\n", d_center);
                    pitch_cmd = 0.0; // EMERGENCY BRAKE
                    yaw_cmd = avoid_speed; // TURN
                }
            }
        }

        // --- TAKEOFF LOCK ---
        if (rel_alt < 0.5) {
            pitch_cmd = 0.0; roll_cmd = 0.0; yaw_cmd = 0.0;
        }
    }

    // === MOTOR MIXING ===
    double alt_error = target_rel_alt - rel_alt;
    double alt_input = k_vertical_p * pow(CLAMP(alt_error + k_vertical_offset, -1.0, 1.0), 3.0);

    double r = k_roll_p * CLAMP(roll, -1.0, 1.0) + gyro_vals[0] + roll_cmd;
    double p = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + gyro_vals[1] + pitch_cmd;
    double y = yaw_cmd;
    double t = k_vertical_thrust + alt_input;

    double fl = t - r + p - y;
    double fr = t + r + p + y;
    double rl = t - r - p + y;
    double rr = t + r - p - y;

    wb_motor_set_velocity(front_left_motor, fl);
    wb_motor_set_velocity(front_right_motor, -fr);
    wb_motor_set_velocity(rear_left_motor, -rl);
    wb_motor_set_velocity(rear_right_motor, rr);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}