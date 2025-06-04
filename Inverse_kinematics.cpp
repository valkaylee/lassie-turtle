#include "controller/inverse_kinematics.h"

#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <cmath>
using namespace std;

#define DEBUG

/**
 * @brief This version of the function interpolates the right flipper’s sweeping and extraction angles 
 *        from a computed starting point to user-specified end points.
 *
 *        It uses a user-provided movement_duration so that when t reaches that duration, the commanded angles
 *        become end_theta and end_gamma.
 *
 * @param turtle_  The turtle object (containing trajectory data and control handles)
 * @param t        The elapsed time (in seconds) since the phase started
 */
void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t) {

    // Geometry parameters (unchanged)
    double l1 = 0.130;             // flipper length (new shorter flipper)
    double turtle_height = 0.079;    // height from flipper to ground (e.g., pivot-to-ground)
    double lower_point = 0.055;      // a lower reference point

    // Use sweeping_range (formerly lateral_angle_range) to compute the horizontal angle (in degrees)
    float horizontal_angle = turtle_.traj_data.sweeping_range * 180 / M_PI;

    // Compute the initial insertion depth (this sets the starting geometry for the extraction angle)
    double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
    if (desierd_insertion_depth > 0.07) {
        desierd_insertion_depth = 0.07;
    }
    double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
        sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point))
        - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
    
    // MODIFIED: Define starting positions for the right flipper.
    // At t = 0, we assume the sweeping angle equals horizontal_angle,
    // and the extraction (adduction) angle is computed from the geometric baseline.
    double start_theta = horizontal_angle;  // MODIFIED
    float right_hori_servo = 0;             // MODIFIED: Base servo offset for the right flipper (could be tuned)
    float extraction_velocity = turtle_.traj_data.extraction_velocity;  // MODIFIED
    double start_gamma = right_hori_servo - extraction_velocity;         // MODIFIED

    // MODIFIED: The user now provides the desired final angles for the right flipper:
    double end_theta = turtle_.traj_data.end_theta;   // desired final sweeping angle // MODIFIED
    double end_gamma = turtle_.traj_data.end_gamma;   // desired final extraction angle // MODIFIED

    // MODIFIED: Assume the user specifies how long this interpolation should take:
    float movement_duration = turtle_.traj_data.movement_duration; // in seconds // MODIFIED

    // MODIFIED: Compute the interpolation fraction (clamped between 0 and 1)
    float corres_t = t / movement_duration; // MODIFIED
    if (corres_t > 1.0f) {
        corres_t = 1.0f;
    }

    // MODIFIED: Linearly interpolate from the starting positions to the user-specified end positions:
    double theta2 = start_theta + (end_theta - start_theta) * corres_t; // MODIFIED
    double gamma2 = start_gamma + (end_gamma - start_gamma) * corres_t; // MODIFIED

    cout << "Interpolation progress: " << corres_t
         << ", theta2 = " << theta2 
         << ", gamma2 = " << gamma2 << endl;

    // Set a gait state value (optional; here we simply mark the phase as active)
    turtle_.turtle_chassis.gait_state = 3;

    // Send commands to the servos.
    // turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = 0; 
    // turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = 0;   
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2; // MODIFIED
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;  // MODIFIED

    // update the commands in radian-turn units (conversion: degrees/360)
    // turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0;
    // turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0;   
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360; // MODIFIED
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;  // MODIFIED
}

/**
 * @brief Bounding gait function that calls the modified controller.
 *
 * @param turtle_  The turtle object.
 * @param t        The current time (in seconds).
 */
void boundingGAIT(turtle& turtle_, float t)
{
    if (use_precomputed_trajectory) {
        // Use pre-computed trajectory
        TrajectoryPoint point = get_trajectory_at_time(t);
        
        // Set motor commands based on trajectory point
        // Assuming gamma and theta are the main control values we want to use
        
        float gamma1 = 0;  // Left flipper adduction angle
        float theta1 = 0;  // Left flipper sweeping angle
        float gamma2 = point.gamma;  // Right flipper adduction angle  
        float theta2 = point.theta;  // Right flipper sweeping angle

        std::cout << "Using precomputed trajectory: t=" << t 
                  << ", theta2=" << theta2 
                  << ", gamma2=" << gamma2 << std::endl;
                  
        // Set a gait state value indicating we're using precomputed trajectory
        turtle_.turtle_chassis.gait_state = 4;
        
        // Send commands to servos 
        turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
        turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
        turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
        turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;

        // In radian
        turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
        turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
        turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
        turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
    } else {
        // Use the original algorithm for trajectory generation
        fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_, t);
    }
}
