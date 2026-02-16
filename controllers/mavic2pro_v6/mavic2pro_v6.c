/*
 * ITERATION 11: Vector Field Avoidance (Smooth Flight)
 * - Removes "Stop-and-Go" logic.
 * - Uses Lidar to calculate Left vs Right density.
 * - Blends avoidance vectors with navigation vectors for continuous flight.
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

// Avoidance Tuning
const double obs_detect_thresh = 3.0;   // Start reacting at 3 meters
const double obs_critical_thresh = 0.8; // Only stop if closer than this
const double avoidance_strength = 1.5;  // How hard to strafe away

const double waypoint_tolerance = 1.5; 

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
  // Flying directly towards the obstacle to test smooth deviation
  path[0].x = -41.5; path[0].y = 4.34; path[0].z = 1.5; 
  total_waypoints = 1; 

  // === DEVICES ===
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  if (lidar > 0) {
      wb_lidar_enable(lidar, timestep);
      wb_lidar_enable_point_cloud(lidar);
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

  printf("Mavic 2 Pro v11: Smooth Vector Avoidance.\n");

  while (wb_robot_step(timestep) != -1) {
    
    // 1. INPUTS
    int key = wb_keyboard_get_key();
    while (key > 0) {
        if (key == 'L') land_mode = !land_mode;
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
    double target_z = 2.0; 
    double pitch_cmd = 0.0;
    double roll_cmd = 0.0;
    double yaw_cmd = 0.0;

    if (land_mode) {
        target_z = 0.0; 
        if (cur_alt < 0.15) {
            for (int m=0; m<4; m++) wb_motor_set_velocity(motors[m], 0.0);
            continue; 
        }
    } 
    else {
        // === STANDARD NAVIGATION ===
        double target_x = path[current_wp_index].x;
        double target_y = path[current_wp_index].y;
        target_z = path[current_wp_index].z;

        double dx = target_x - cur_x;
        double dy = target_y - cur_y;
        double dist_to_wp = sqrt(dx*dx + dy*dy);

        // Check Arrival
        if (dist_to_wp < waypoint_tolerance && fabs(cur_alt - target_z) < 0.5) {
            printf(">> WP %d Reached.\n", current_wp_index);
            current_wp_index++;
            if (current_wp_index >= total_waypoints) current_wp_index = total_waypoints - 1; 
        }

        // Base Heading Calculation
        double angle_to_target = atan2(dy, dx); 
        double heading_error = angle_to_target - yaw;
        while (heading_error > PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;

        // Base Forward Motion (Seek the waypoint)
        // We pitch forward (-negative pitch) based on alignment
        if (fabs(heading_error) < 1.0) { 
             pitch_cmd = -2.0; // Standard cruise speed
        } else {
             pitch_cmd = 0.0;  // Turn in place if facing wrong way
        }
        
        yaw_cmd = k_yaw_p * heading_error;

        // === LIDAR VECTOR FIELD AVOIDANCE ===
        double avoidance_roll = 0.0;
        double avoidance_yaw = 0.0;
        bool critical_stop = false;

        if (lidar > 0) {
            const float *lidar_values = wb_lidar_get_range_image(lidar);
            if (lidar_values) {
                int w = wb_lidar_get_horizontal_resolution(lidar);
                int mid = w / 2;

                // Calculate average distance in Left and Right sectors
                double left_sum = 0, right_sum = 0;
                int left_count = 0, right_count = 0;
                double min_dist = 100.0; // track closest point

                // Scan center 60 degrees (approx)
                // Assuming lidar scan is Left(max) to Right(0) or vice versa. 
                // We split indices 0..mid (Right?) and mid..w (Left?)
                // Adjust indices based on your specific Lidar model rotation.
                // Here we assume standard: index 0 is right, index w is left.
                
                for (int i = 0; i < w; i++) {
                     float range = lidar_values[i];
                     if (isinf(range)) range = 10.0; // Cap infinite
                     
                     if (range < min_dist) min_dist = range;

                     // Only care about obstacles in front (< obs_detect_thresh)
                     if (range < obs_detect_thresh) {
                         if (i < mid) { // Right Side
                             right_sum += range;
                             right_count++;
                         } else {       // Left Side
                             left_sum += range;
                             left_count++;
                         }
                     }
                }

                // If obstacle detected
                if (min_dist < obs_detect_thresh) {
                    
                    // CRITICAL SAFETY: Only stop if we are about to crash
                    if (min_dist < obs_critical_thresh) {
                        critical_stop = true;
                    } else {
                        // SMOOTH AVOIDANCE
                        // If Left is closer (smaller sum/avg), we must go Right.
                        // If Right is closer, we must go Left.
                        
                        double left_avg = (left_count > 0) ? left_sum/left_count : 10.0;
                        double right_avg = (right_count > 0) ? right_sum/right_count : 10.0;

                        // Calculate avoidance intensity based on proximity
                        // The closer we are, the stronger the push.
                        double urgency = (obs_detect_thresh - min_dist) / obs_detect_thresh; 
                        
                        if (left_avg < right_avg) {
                            // Obstacle is on the Left -> Strafe Right
                             avoidance_roll = avoidance_strength * urgency; // Positive roll = Right
                             avoidance_yaw = -0.5 * urgency; // Turn slightly Right
                             printf(">> AVOID: Sliding Right (Dist: %.2f)\n", min_dist);
                        } else {
                            // Obstacle is on the Right -> Strafe Left
                             avoidance_roll = -avoidance_strength * urgency; // Negative roll = Left
                             avoidance_yaw = 0.5 * urgency; // Turn slightly Left
                             printf(">> AVOID: Sliding Left (Dist: %.2f)\n", min_dist);
                        }

                        // Reduce forward speed slightly while avoiding, but don't stop
                        pitch_cmd *= 0.7; 
                    }
                }
            }
        }

        // === COMBINE VECTORS ===
        if (critical_stop) {
             pitch_cmd = 0.0; // Hover
             roll_cmd = 0.0;
             printf(">> CRITICAL BRAKE!\n");
        } else {
             // Navigation + Avoidance
             // We add the avoidance vector to the navigation vector
             roll_cmd += avoidance_roll;
             yaw_cmd += avoidance_yaw;
             
             // Clamp limits
             pitch_cmd = CLAMP(pitch_cmd, -2.0, 2.0);
             roll_cmd = CLAMP(roll_cmd, -1.0, 1.0);
             yaw_cmd = CLAMP(yaw_cmd, -1.0, 1.0);
        }

        // Damping Yaw
        yaw_cmd -= k_yaw_d * yaw_velocity;
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