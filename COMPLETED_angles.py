import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parameters
L = 1.0
alpha_deg_range = 45
beta_deg_range = 45

# Convert angles to radians
alpha_limit = np.radians(alpha_deg_range)
beta_limit = -np.radians(beta_deg_range)

# Generate spherical cap points 
alpha = np.linspace(-alpha_limit, alpha_limit, 100)
beta = np.linspace(beta_limit, 0, 100)
alpha, beta = np.meshgrid(alpha, beta)
x = L * np.sin(alpha)
y = L * np.cos(alpha) * np.cos(beta)
z = L * np.cos(alpha) * np.sin(beta)

# Plotting the workspace 
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor('white')

# Plot spherical cap 
ax.plot_surface(x, y, z, color='cyan', edgecolor='none', alpha=0.2)

# Origin point
start_point = (0, 0, 0)
ax.scatter(*start_point, color='red', s=150, label="Origin")

# Function to calculate points on spherical cap 
def calculate_point(alpha_angle, beta_angle):
    x = L * np.sin(alpha_angle)
    y = L * np.cos(alpha_angle) * np.cos(beta_angle)
    z = L * np.cos(alpha_angle) * np.sin(beta_angle)
    return (x, y, z)

# Generate eight angled trajectories with different angles
angles_deg = [0, 10, 20, 30, 40, 50, 68] 
colors = ['yellow', 'green', 'purple', 'blue', 'orange', 'brown', 'cyan']  # Colors for each trajectory

for i, angle in enumerate(angles_deg):
    # Convert angle to radians
    alpha_offset = np.radians(angle)
    beta_offset = np.radians(angle)
    
    # --------------------------
    # Vertical Trajectory
    # --------------------------
    # Anchor at the top right on the spherical cap: (alpha_limit, 0)
    start_v_angles = (alpha_limit, 0)
    # Endpoint: shift left in alpha and move to the bottom (beta_limit)
    end_v_angles = (alpha_limit - alpha_offset, beta_limit)
    
    # Calculate the trajectory points
    start_v_point = calculate_point(*start_v_angles)
    end_v_point = calculate_point(*end_v_angles)
    
    num_points = 100
    alpha_vals = np.linspace(start_v_angles[0], end_v_angles[0], num_points)
    beta_vals = np.linspace(start_v_angles[1], end_v_angles[1], num_points)
    x_vals = L * np.sin(alpha_vals)
    y_vals = L * np.cos(alpha_vals) * np.cos(beta_vals)
    z_vals = L * np.cos(alpha_vals) * np.sin(beta_vals)
    
    ax.plot(x_vals, y_vals, z_vals, linestyle="-", linewidth=2, color=colors[i],
            label=f"Vertical Angle {angle}°")
    ax.scatter(*start_v_point, color='green', s=100)
    ax.scatter(*end_v_point, color='yellow', s=100)
    
    # --------------------------
    # Horizontal Trajectory
    # --------------------------
    # Anchor at the top right on the spherical cap: (alpha_limit, 0)
    start_h_angles = (alpha_limit, 0)
    # Endpoint: shift fully left in alpha and adjust beta; 
    # Clamp beta to not go below beta_limit so it stays on the cap.
    new_beta = max(-beta_offset, beta_limit)
    end_h_angles = (-alpha_limit, new_beta)
    
    # Calculate the trajectory points
    start_h_point = calculate_point(*start_h_angles)
    end_h_point = calculate_point(*end_h_angles)
    
    alpha_vals = np.linspace(start_h_angles[0], end_h_angles[0], num_points)
    beta_vals = np.linspace(start_h_angles[1], end_h_angles[1], num_points)
    x_vals = L * np.sin(alpha_vals)
    y_vals = L * np.cos(alpha_vals) * np.cos(beta_vals)
    z_vals = L * np.cos(alpha_vals) * np.sin(beta_vals)
    
    ax.plot(x_vals, y_vals, z_vals, linestyle="-", linewidth=2, color=colors[i],
            label=f"Horizontal Angle {angle}°")
    ax.scatter(*start_h_point, color='blue', s=100)
    ax.scatter(*end_h_point, color='magenta', s=100)

# Label axes and set the title
ax.set_xlabel("X", color='white')
ax.set_ylabel("Y", color='white')
ax.set_zlabel("Z", color='white')
ax.set_title("Workspace with Eight Angled L-Shaped Trajectories (All on Spherical Cap)", color='white')
# ax.legend(facecolor='black', edgecolor='white', labelcolor='white', prop={'size': 8})

plt.show()
