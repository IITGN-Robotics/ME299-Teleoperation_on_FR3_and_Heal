#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# Initialize global history buffers for each coordinate
max_len = 200  # Max number of points to show
x_data, y_data, z_data = deque(maxlen=max_len), deque(maxlen=max_len),deque(maxlen=max_len)
time_data = deque(maxlen=max_len)

# Callback to process incoming ArUco position
def marker_callback(msg):
    t = rospy.get_time()
    x, y, z = msg.data

    # Append new values to the buffers
    time_data.append(t)
    x_data.append(x)
    y_data.append(y)
    z_data.append(z)

# Plotting function
def live_plot():
    plt.ion()
    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label='X')
    line_y, = ax.plot([], [], label='Y')
    line_z, = ax.plot([], [], label='Z')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position')
    ax.set_title('Real-Time ArUco Marker Position')
    ax.legend()
    ax.grid(True)

    while not rospy.is_shutdown():
        if time_data:
            # Update data in the plot
            line_x.set_data(time_data, x_data)
            line_y.set_data(time_data, y_data)
            line_z.set_data(time_data, z_data)

            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    rospy.init_node('aruco_plotter', anonymous=True)
    rospy.Subscriber('/aruco_position', Float32MultiArray, marker_callback)

    try:
        live_plot()
    except rospy.ROSInterruptException:
        pass
