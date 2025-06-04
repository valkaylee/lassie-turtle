#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "controller/inverse_kinematics.h"
// shoudl time be the same for any sequence of points? 

// Struct to hold a single trajectory point
struct TrajectoryPoint {
    double time;     // Time stamp (seconds from start)
    double gamma;    // Gamma angle (degrees)
    double theta;    // Theta angle (degrees)
};

// Struct to hold a complete trajectory
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    size_t current_index;
    double total_duration;
};

// trajectory object to store precomputed values
Trajectory precomputed_trajectory;

// Flag for whether to use precomputed trajectory
bool use_precomputed_trajectory = false;

// Interpolate between two waypoints
TrajectoryPoint interpolate_waypoints(const TrajectoryPoint& start, const TrajectoryPoint& end, double t_normalized) {
    TrajectoryPoint result;
    
    result.time = start.time + (end.time - start.time) * t_normalized;
    result.gamma = start.gamma + (end.gamma - start.gamma) * t_normalized;
    result.theta = start.theta + (end.theta - start.theta) * t_normalized;
    
    return result;
}

// Generate a trajectory from a series of waypoints
void generate_trajectory(const std::vector<TrajectoryPoint>& waypoints, double sampling_rate = 0.01) {
    precomputed_trajectory.points.clear();
    precomputed_trajectory.current_index = 0;
    
    // Process each waypoint segment
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        double segment_duration = waypoints[i+1].time - waypoints[i].time;
        int steps = std::ceil(segment_duration / sampling_rate); // find number of steps in each segemnt duration 
        
        // Generate points for this segment
        for (int j = 0; j < steps; j++) {
            double t_normalized = static_cast<double>(j) / steps;
            TrajectoryPoint point = interpolate_waypoints(waypoints[i], waypoints[i+1], t_normalized);
            precomputed_trajectory.points.push_back(point); // add interpolated point to precomputed_trajectory
        }
    }
    
    // Add the final waypoint
    if (!waypoints.empty()) {
        precomputed_trajectory.points.push_back(waypoints.back());
        precomputed_trajectory.total_duration = waypoints.back().time;
    }
    
    std::cout << "Generated trajectory with " << precomputed_trajectory.points.size() << " points" << std::endl;
}

// Save trajectory to CSV file for debugging 
// void save_trajectory_to_csv(const std::string& filename) {
//     std::ofstream outfile(filename);
    
//     if (!outfile.is_open()) {
//         std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
//         return;
//     }
    
//     // Write header
//     outfile << "time,alpha,beta,gamma,theta" << std::endl;
    
//     // Write data points
//     for (const auto& point : precomputed_trajectory.points) {
//         outfile << point.time << "," 
//                 << point.alpha << "," 
//                 << point.beta << "," 
//                 << point.gamma << "," 
//                 << point.theta << std::endl;
//     }
    
//     outfile.close();
//     std::cout << "Trajectory saved to " << filename << std::endl;
// }

// Function to load trajectory from CSV file
// bool load_trajectory_from_csv(const std::string& filename) {
//     std::ifstream infile(filename);
    
//     if (!infile.is_open()) {
//         std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
//         return false;
//     }
    
//     precomputed_trajectory.points.clear();
//     precomputed_trajectory.current_index = 0;
    
//     // Skip header
//     std::string line;
//     std::getline(infile, line);
    
//     // Read data points
//     while (std::getline(infile, line)) {
//         TrajectoryPoint point;
//         std::stringstream ss(line);
//         std::string value;
        
//         // Parse time
//         std::getline(ss, value, ',');
//         point.time = std::stod(value);
        
//         // Parse alpha
//         std::getline(ss, value, ',');
//         point.alpha = std::stod(value);
        
//         // Parse beta
//         std::getline(ss, value, ',');
//         point.beta = std::stod(value);
        
//         // Parse gamma
//         std::getline(ss, value, ',');
//         point.gamma = std::stod(value);
        
//         // Parse theta
//         std::getline(ss, value);
//         point.theta = std::stod(value);
        
//         precomputed_trajectory.points.push_back(point);
//     }
    
//     if (!precomputed_trajectory.points.empty()) {
//         precomputed_trajectory.total_duration = precomputed_trajectory.points.back().time;
//     }
    
//     std::cout << "Loaded trajectory with " << precomputed_trajectory.points.size() << " points" << std::endl;
//     use_precomputed_trajectory = true;
    
//     return true;
// }

// Example setup for a circular trajectory
// void setup_trajectory_example(turtle& turtle_) {
//     // Create waypoints for a sample trajectory
//     std::vector<TrajectoryPoint> waypoints;
    
//     // Define some waypoints for a demo trajectory
//     // This could be replaced with your actual waypoints from user input or planning
    
//     double max_time = 3.0;  // Total duration of the trajectory
//     int num_waypoints = 10; // Number of waypoints to generate
    
//     for (int i = 0; i < num_waypoints; i++) {
//         double t = max_time * i / (num_waypoints - 1);
//         double progress = static_cast<double>(i) / (num_waypoints - 1);
        
//         TrajectoryPoint point;
//         point.time = t;
        
//         // Example: Generate a simple sinusoidal trajectory
//         // You would replace this with your desired waypoints
//         double angle_range = turtle_.traj_data.lateral_angle_range;
//         double depth_range = turtle_.traj_data.insertion_depth;
        
//         // Example angles
//         point.gamma = turtle_.traj_data.start_gamma + progress * (turtle_.traj_data.end_gamma - turtle_.traj_data.start_gamma);
//         point.theta = turtle_.traj_data.start_theta + progress * (turtle_.traj_data.end_theta - turtle_.traj_data.start_theta);
        
//         // Add a sinusoidal component for a curved path
//         point.gamma += 0.1 * sin(progress * 2 * M_PI);
        
//         // Alpha and beta are placeholders - you would compute these based on your IK
//         point.alpha = 0.0;
//         point.beta = 0.0;
        
//         waypoints.push_back(point);
//     }
    
//     // Generate trajectory from waypoints
//     generate_trajectory(waypoints);
    
//     // Save to file
//     save_trajectory_to_csv("turtle_trajectory.csv");
// }

// determine what flipper angles should look like at a given time 
TrajectoryPoint get_trajectory_at_time(double t) {
    // Handle loop or clamping behavior when time exceeds trajectory duration
    double trajectory_time = fmod(t, precomputed_trajectory.total_duration);
    
    // Find the appropriate segment in trajectory
    size_t i = 0;
    while (i < precomputed_trajectory.points.size() - 1 && 
           precomputed_trajectory.points[i+1].time < trajectory_time) {
        i++;
    }
    
    // if time is past the last point
    if (i >= precomputed_trajectory.points.size() - 1) {
        return precomputed_trajectory.points.back();
    }
    
    // where are we are between points i and i+1
    double segment_duration = precomputed_trajectory.points[i+1].time - precomputed_trajectory.points[i].time;
    double segment_progress = 0.0;
    
    if (segment_duration > 0) {
        segment_progress = (trajectory_time - precomputed_trajectory.points[i].time) / segment_duration;
    }
    
    // Interpolate between points
    return interpolate_waypoints(
        precomputed_trajectory.points[i], 
        precomputed_trajectory.points[i+1], 
        segment_progress);
}