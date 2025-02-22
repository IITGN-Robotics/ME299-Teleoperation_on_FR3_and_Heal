'''
Using this code we could give joint commands to the robot using the joint controller
'''
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import time  # To track the elapsed time

def send_velocity_command(command, duration=2):
    """
    Publishes the desired velocity command to the robot's velocity controller for a specified duration.

    :param command: List of velocities for the robot joints (Float64MultiArray data).
    :param duration: Time in seconds for which the command should be sent.
    """
    rospy.init_node('velocity_command_publisher', anonymous=True)
    pub = rospy.Publisher('/velocity_controller/command', Float64MultiArray, queue_size=10)

    # Wait for the publisher to connect to the topic
    rospy.loginfo("Waiting for connection to /velocity_controller/command topic...")
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    msg = Float64MultiArray()
    msg.data = command

    rospy.loginfo(f"Starting to publish velocity command: {command} for {duration} seconds.")
    start_time = time.time()  # Record the current time

    # Publish the command repeatedly for the specified duration
    while time.time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(0.1)  # Publish at a 10 Hz rate

    # Stop the robot by sending a zero velocity command
    rospy.loginfo("Stopping the robot by sending zero velocity.")
    msg.data = [0] * len(command)  # Zero velocity for all joints
    pub.publish(msg)
    rospy.loginfo("Command sent successfully!")


if __name__ == "__main__":
    try:
        # Example: Set desired velocities for the joints
        desired_velocity = [0.1, 0, 0, 0, 0, 0]  # Modify as per your requirement
        run_duration = 2  # Run the movement for 2 seconds
        send_velocity_command(desired_velocity, run_duration)
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")
