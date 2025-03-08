#!/usr/bin/env python3

import rospy
from pynput import keyboard
from std_msgs.msg import Float64MultiArray

# Define key mappings
v = 0.1
key_mapping = {
    'q': [v, 0, 0, 0, 0, 0],  
    'a': [-v, 0, 0, 0, 0, 0],  
    'w': [0, v, 0, 0, 0, 0],
    's': [0, -v, 0, 0, 0, 0],
    'e': [0, 0, v, 0, 0, 0],  
    'd': [0, 0, -v, 0, 0, 0], 
    'r': [0, 0, 0, v, 0, 0],
    'f': [0, 0, 0, -v, 0, 0],
    't': [0, 0, 0, 0, v, 0],
    'g': [0, 0, 0, 0, -v, 0],
    'y': [0, 0, 0, 0, 0, v],
    'h': [0, 0, 0, 0, 0, -v],

}

# Initialize active command
active_command = [0, 0, 0, 0, 0, 0]
pressed_keys = set()

def update_active_command():
    """ Updates the active command based on pressed keys. """
    global active_command
    active_command = [0, 0, 0, 0, 0, 0]

    for key in pressed_keys:
        if key in key_mapping:
            active_command = [sum(x) for x in zip(active_command, key_mapping[key])]
        elif hasattr(key, 'char') and key.char in key_mapping:
            active_command = [sum(x) for x in zip(active_command, key_mapping[key.char])]

def on_press(key):
    """ Handles key press events. """
    global pressed_keys
    pressed_keys.add(key)
    update_active_command()

def on_release(key):
    """ Handles key release events. """
    global pressed_keys
    if key in pressed_keys:
        pressed_keys.remove(key)
    update_active_command()

    if key == keyboard.Key.esc:
        return False  # Stop listener

def keyboard_control():
    rospy.init_node('keyboard_velocity_control', anonymous=True)
    pub = rospy.Publisher('/velocity_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(50)  # Adjust update rate for responsiveness

    print("Use WASD or Arrow Keys to move, Press 'Esc' to quit.")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = active_command
        pub.publish(msg)
        print(f"Sent command: {active_command}", end='\r')
        rate.sleep()

    listener.stop()

if __name__ == "__main__":
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")
