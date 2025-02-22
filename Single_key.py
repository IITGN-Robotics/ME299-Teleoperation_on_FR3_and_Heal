'''
The code below takes input from the keyboard regarding the commands for each joint of the heal robot in a node called`keyboard_velocity_control` and publishes it onto a 
topic called `/velocity_controller/command`, which is subscribed to by the robot's node. This node reads the commands and translates them into the physical motion of the robot.
The code maps `w`, `s`, `a`, `d`, `q`, and `e` to unique joint motions. Only one key can be pressed at a time, and the robot moves only when a key is pressed.
'''
#!/usr/bin/env python

import rospy
import curses
from std_msgs.msg import Float64MultiArray

# Define key mappings
key_mapping = {
    'w': [0.1, 0, 0, 0, 0, 0],  # anticlock wise motion of the first joint (moving towards left)
    's': [-0.1, 0, 0, 0, 0, 0], # clock wise motion of the first joint (moving towards right)
    'a': [0, 0.1, 0, 0, 0, 0],  # anticlock wise motion of the second joint (moving downwards)
    'd': [0, -0.1, 0, 0, 0, 0], # clock wise motion of the second joint (moving upwards)
    'q': [0, 0, 0.1, 0, 0, 0],  # anticlock wise motion of the third joint (moving forward)
    'e': [0, 0, -0.1, 0, 0, 0], # clock wise motion of the third joint (moving backwards)
}

def keyboard_control(stdscr):
    """ Captures keyboard input and moves only while a key is pressed. """
    rospy.init_node('keyboard_velocity_control', anonymous=True)
    pub = rospy.Publisher('/velocity_controller/command', Float64MultiArray, queue_size=10)

    stdscr.nodelay(True)  # Non-blocking input mode
    stdscr.clear()
    stdscr.addstr("Use WASD to move, Q/E to rotate, hold key to move, release to stop. Press 'Esc' to quit.\n")

    active_command = [0, 0, 0, 0, 0, 0]
    
    while not rospy.is_shutdown():
        key = stdscr.getch()  # Get keyboard input
        
        if key == 27:  # Escape key to quit
            break

        if key in map(ord, key_mapping.keys()):
            active_command = key_mapping[chr(key)]
        else:
            active_command = [0, 0, 0, 0, 0, 0]  # Stop immediately when no valid key is pressed

        msg = Float64MultiArray()
        msg.data = active_command
        pub.publish(msg)

        try:
            stdscr.addstr(1, 0, f"Sent command: {active_command}    ")  # Display command
            stdscr.refresh()
        except curses.error:
            pass  # Ignore curses display errors

        rospy.Rate(25).sleep()  # Faster response loop

    # Stop the robot before exiting
    stop_msg = Float64MultiArray()
    stop_msg.data = [0, 0, 0, 0, 0, 0]
    pub.publish(stop_msg)

# Run the curses-based keyboard control
if __name__ == "__main__":
    try:
        curses.wrapper(keyboard_control)  # Initializes and runs curses
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")
