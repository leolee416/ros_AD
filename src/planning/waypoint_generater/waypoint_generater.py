"""
 *  Path Interpolation and Visualization Script
 *
 *  This script implements a method to visualize a set of path points for a vehicle,
 *  using both straight lines and spline curves for interpolation where necessary.
 *  It interpolates points that are farther apart than a specified threshold (20 meters),
 *  and uses spline interpolation for sequences of points that are close together.
 *  The resulting path is plotted using matplotlib, showing both the original and
 *  interpolated points.
 *
 *  Author: Jincheng Pan
 *  Date: 27.07.2024
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

parts = [
    np.array([[-20.00, -60.40], [-25.00, -59.50], [-30.00, -58.00], [-35.00, -55.00], [-40.00, -50.00], 
              [-45.00, -45.00], [-50.00, -35.00], [-52.00, -15.00], [-52.46, 39.64], [-52.50, 117.50], 
              [-48.30, 121.30], [-10.26, 122.87], [-10.00, 122.87]]),
    np.array([[-7.21, 123.00], [-7.00, 125.00], [-6.80, 130.00], [-6.60, 135.00], [-6.50, 140.00], [-6.30, 150.00], 
              [-6.00, 160.00], [-5.50, 180.00], [-4.50, 190.00], [-3.56, 223.10]]),
    np.array([[-4.10, 224.50], [-4.50, 225.00], [-5.00, 225.50], [-6.00, 226.00], [-10.00, 229.00], [-15.00, 229.50], 
              [-20.00, 229.00], [-25.00, 228.50], [-30.00, 228.00], [-35.00, 227.50], [-40.00, 225.00], [-45.00, 220.00], 
              [-50.00, 210.00], [-52.00, 200.00], [-52.20, 121.16], [-52.04, 50.51], [-48.04, 46.51], [-10.00, 45.32], [-9.56, 45.32]]),
    np.array([[-8.01, 44.37], [-7.00, 40.00], [-6.50, 37.00], [-6.00, 34.00], [-5.00, 30.00], [-4.50, 20.00], [-4.00, 10.00], 
              [-3.50, -10.00], [-3.00, -30.00], [-2.30, -59.00], [-2.30, -59.50]]),
    np.array([[-2.46, -63.00], [1.00, -63.50], [4.74, -63.90], [4.94, -63.90]])
]

# Calculate the distance between two points
def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Generate interpolation points
def interpolate_points(p1, p2, step=20):
    dist = distance(p1, p2)
    num_points = int(dist // step)
    if num_points == 0:
        return []

    points = []
    for i in range(1, num_points + 1):
        t = i * step / dist
        x = (1 - t) * p1[0] + t * p2[0]
        y = (1 - t) * p1[1] + t * p2[1]
        points.append([x, y])
    return np.array(points)

# Plot points with lines and curves
def plot_points_with_lines_and_curves(parts):
    plt.figure(figsize=(14, 10))
    all_points = []
    interpolated_points = []

    for idx, points in enumerate(parts):
        part_points = [points[0]]
        i = 0
        while i < len(points) - 1:
            if distance(points[i], points[i + 1]) >= 20:
                plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], 'k-')
                inter_points = interpolate_points(points[i], points[i + 1], step=20)
                interpolated_points.append(inter_points)
                part_points.append(points[i + 1])
            else:
                j = i + 1
                group_points = [points[i]]
                while j < len(points) and distance(points[j - 1], points[j]) < 20:
                    group_points.append(points[j])
                    j += 1

                group_points = np.array(group_points)
                if len(group_points) > 3:  # If the number of points is greater than 3, use spline interpolation
                    tck, u = splprep([group_points[:, 0], group_points[:, 1]], s=0)
                    u_new = np.linspace(u.min(), u.max(), 100)
                    x_new, y_new = splev(u_new, tck, der=0)
                    plt.plot(x_new, y_new, 'r-')
                    part_points.extend(group_points[1:])
                else:
                    plt.plot(group_points[:, 0], group_points[:, 1], 'k-')
                    part_points.extend(group_points[1:])
                
                i = j - 2

            i += 1
        
        all_points.extend(part_points)
        
        # Draw the connection between the last point of the current part and the first point of the next part
        if idx < len(parts) - 1:
            next_part = parts[idx + 1]
            if distance(part_points[-1], next_part[0]) <= 20:
                plt.plot([part_points[-1][0], next_part[0][0]], [part_points[-1][1], next_part[0][1]], 'k-')
            else:
                plt.plot([part_points[-1][0], next_part[0][0]], [part_points[-1][1], next_part[0][1]], 'g--')

    all_points = np.array(all_points)
    plt.plot(all_points[:, 0], all_points[:, 1], 'bo')

    # Plot interpolated points
    for inter_points in interpolated_points:
        if len(inter_points) > 0:
            plt.plot(inter_points[:, 0], inter_points[:, 1], 'go')

    plt.title("Connected Points with Lines and Curves")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    
    plt.arrow(min(all_points[:,0]) - 50, min(all_points[:,1]) - 50, 40, 0, head_width=10, head_length=10, fc='k', ec='k')
    plt.arrow(min(all_points[:,0]) - 50, min(all_points[:,1]) - 50, 0, 40, head_width=10, head_length=10, fc='k', ec='k')
    plt.text(min(all_points[:,0]) - 50 + 45, min(all_points[:,1]) - 50, 'X', fontsize=12)
    plt.text(min(all_points[:,0]) - 50, min(all_points[:,1]) - 50 + 45, 'Y', fontsize=12)

    plt.show()

    return all_points

all_points = plot_points_with_lines_and_curves(parts)

# Define the directory and filename
current_dir = os.path.dirname(os.path.abspath(__file__))
filename = 'waypoints.txt'
file_path = os.path.join(current_dir, "waypointsdata/", filename)

# Save all points to a txt file
np.savetxt(file_path, all_points, fmt='%.2f', delimiter=' ', header='X Y', comments='')

print(f"All points saved to {file_path}")
