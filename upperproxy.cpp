
#include "proxy/upperproxy.h"

/**
 * upperproxy - class to collect robot's information and trajectories from path
 * planning and decision making part. 
 * agile taur.
 */

namespace turtle_namespace{
namespace control{

// Add new subscription in constructor
upperproxy::upperproxy(std::string name) : Node(name){
    std::cout<<"Traveler Upper Proxy established"
                <<std::endl;
    GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/Gui_information", 10, std::bind(&upperproxy::handle_gui, this, _1));
    trajectory_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/trajectory_waypoints", 10, std::bind(&upperproxy::handle_trajectory_points, this, _1));
}

// Handle trajectory points from GUI or other source
void upperproxy::handle_trajectory_points(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Clear existing waypoints
    waypoints.clear();
    
    // Check if message has valid data (at least one waypoint of 3 values: time, gamma, theta)
    if (msg->data.size() % 3 != 0 || msg->data.size() < 3) {
        std::cout << "Invalid trajectory waypoints data received" << std::endl;
        return;
    }
    
    // Parse waypoints from message
    for (size_t i = 0; i < msg->data.size(); i += 3) {
        TrajectoryWaypoint wp;
        wp.time = msg->data[i];
        wp.gamma = msg->data[i+1];
        wp.theta = msg->data[i+2];
        waypoints.push_back(wp);
    }
    
    std::cout << "Received " << waypoints.size() << " trajectory waypoints" << std::endl;
    
    // Generate trajectory from waypoints
    GenerateTrajectoryFromWaypoints();
}

// Generate trajectory from waypoints
void upperproxy::GenerateTrajectoryFromWaypoints() {
    if (waypoints.empty()) {
        std::cout << "No waypoints available to generate trajectory" << std::endl;
        return;
    }
    
    // Convert to the format expected by the trajectory generator
    std::vector<TrajectoryPoint> traj_waypoints;
    
    for (const auto& wp : waypoints) {
        TrajectoryPoint point;
        point.time = wp.time;
        point.gamma = wp.gamma;
        point.theta = wp.theta;
        point.alpha = 0.0;  // placeholder
        point.beta = 0.0;   // placeholder
        traj_waypoints.push_back(point);
    }
    
    // Generate and save trajectory
    generate_trajectory(traj_waypoints);
    // save_trajectory_to_csv("turtle_trajectory.csv");
    
    // Set flag to use precomputed trajectory
    use_precomputed_trajectory = true;
    
    std::cout << "Generated trajectory from " << waypoints.size() << " waypoints" << std::endl;
}

void upperproxy::handle_gui(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    // int len = msg->data.size();
    turtle_inter_.turtle_gui.start_flag = msg->data[0]; 
    turtle_inter_.turtle_gui.drag_traj = msg->data[1];
    turtle_inter_.traj_data.sweeping_range = msg->data[2];
    turtle_inter_.traj_data.insertion_depth = msg->data[3];
    turtle_inter_.traj_data.penetration_velocity = msg->data[4];
    turtle_inter_.traj_data.sweeping_velocity = msg->data[5];
    turtle_inter_.traj_data.extraction_velocity = msg->data[6];
    turtle_inter_.traj_data.swing_velocity = msg->data[7];
    // MODIFIED: Check if new start/end coordinates are provided in the message.
    turtle_inter_.traj_data.start_gamma = msg->data[8];  // MODIFIED: new start_x (in radians)
    turtle_inter_.traj_data.start_theta = msg->data[9];  // MODIFIED: new start_y (in radians)
    turtle_inter_.traj_data.end_gamma   = msg->data[10]; // MODIFIED: new end_x (in radians)
    turtle_inter_.traj_data.end_theta  = msg->data[11]; // MODIFIED: new end_y (in radians)
}

void upperproxy::UpdateGuiCommand(turtle &turtle_){
    turtle_.turtle_gui = turtle_inter_.turtle_gui;
    turtle_.traj_data = turtle_inter_.traj_data;
}
void upperproxy::PublishStatusFeedback(turtle &turtle_){
    if(turtle_.turtle_gui.status_update_flag == true){
        auto message = std_msgs::msg::Float64MultiArray();
        // std::cout <<  message.data[message.data.size() - 1] << std::endl;
        GUI_publisher->publish(message);
        turtle_.turtle_gui.status_update_flag = false;
    }
    
}

} //namespace control
} //namespace turtle_namespace
