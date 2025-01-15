"""
 *  Error Data Visualization Script
 *
 *  This script reads error data for speed and steering from a file and visualizes
 *  the errors over time using matplotlib. The error data is assumed to be stored
 *  in a text file located in the .ros directory of the user's home directory.
 *  The script plots the speed error and steering error on separate subplots.
 *
 *  Author: Jincheng Pan
 *  Date: 27.07.2024
"""

import matplotlib.pyplot as plt
import os

def read_error_data(filename):
    times = []
    speed_errors = []
    steering_errors = []
    
    with open(filename, 'r') as file:
        for line in file:
            parts = line.split()
            if len(parts) == 3:
                times.append(float(parts[0]))
                speed_errors.append(float(parts[1]))
                steering_errors.append(float(parts[2]))
                
    return times, speed_errors, steering_errors

def plot_errors(times, speed_errors, steering_errors):
    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(times, speed_errors, label='Speed Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed Error')
    plt.title('Speed Error over Time')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, steering_errors, label='Steering Error', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Steering Error')
    plt.title('Steering Error over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    home_directory = os.path.expanduser("~")
    filename = os.path.join(home_directory, ".ros", "error_data.txt")
    times, speed_errors, steering_errors = read_error_data(filename)
    plot_errors(times, speed_errors, steering_errors)
