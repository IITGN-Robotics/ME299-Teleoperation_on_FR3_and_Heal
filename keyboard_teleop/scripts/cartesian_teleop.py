#!/usr/bin/env python3

import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
from pynput import keyboard
import threading

# Initialize ROS Node
rospy.init_node("jacobian_computation")

# Create a publisher for joint velocities
joint_velocity_pub = rospy.Publisher("/velocity_controller/command", Float64MultiArray, queue_size=10)

# Key mapping for Cartesian velociqqqqwwqaaaty control
v = 0.035
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

# Track key states safely
key_states = {k: False for k in key_mapping}

# Mutex for thread-safe access
key_lock = threading.Lock()

# Load URDF and Get KDL Chain
def get_robot_kdl_chain():
    if not rospy.has_param("/robot_description"):
        rospy.logerr("URDF not found on parameter server!")
        return None, None

    robot = URDF.from_parameter_server()
    success, tree = treeFromParam("/robot_description")
    
    if not success:
        rospy.logerr("Failed to parse URDF into KDL tree!")
        return None, None

    base_link = robot.get_root()
    
    end_effector = "tool"  # Change this based on your robot's URDF
    
    if end_effector not in [l.name for l in robot.links]:
        rospy.logerr("End-effector link not found in URDF!")
        return None, None
    
    chain = tree.getChain(base_link, end_effector)
    return chain, chain.getNrOfJoints()

# Compute the Jacobian Matrixe
def compute_jacobian(joint_angles):
    chain, num_joints = get_robot_kdl_chain()
    if chain is None:
        rospy.logerr("KDL chain is not available!")
        return None

    jac_solver = kdl.ChainJntToJacSolver(chain)
    jnt_array = kdl.JntArray(num_joints)
    for i in range(num_joints):
        jnt_array[i] = joint_angles[i]

    jacobian = kdl.Jacobian(num_joints)  # Create an empty Jacobian matrix
    jac_solver.JntToJac(jnt_array, jacobian)  # Compute the Jacobian

    J_np = np.zeros((jacobian.rows(), jacobian.columns()))  # Initialize NumPy array
    for i in range(jacobian.rows()):
        for j in range(jacobian.columns()):
            J_np[i, j] = jacobian[i, j]  # Copy values correctly

    return J_np  # Return Jacobian as NumPy array

# Key press handling
def on_press(key):
    try:
        with key_lock:
            if key.char in key_states:
                key_states[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        with key_lock:
            if key.char in key_states:
                key_states[key.char] = False
    except AttributeError:
        pass

# Start keyboard listener in a separate thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Main Function to Compute and Publish Joint Velocities
def compute_and_publish():
    rospy.loginfo("Jacobian Computation and Velocity Publishing Started!")
    rate = rospy.Rate(800)
    velocity_msg = Float64MultiArray()
    
    while not rospy.is_shutdown():
        try:
            joint_state_msg = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
            joint_angles = np.array(joint_state_msg.position)
        except rospy.ROSException:
            rospy.logerr("Failed to get joint states. Check if the topic is publishing!")
            continue

        # Read key states safely
        with key_lock:
            pressed_keys = [k for k, v in key_states.items() if v]

        if not pressed_keys:
            velocity_msg.data = [0] * 6  # Stop motion when no key is pressed
        else:
            cartesian_vel = np.zeros(6)
            for k in pressed_keys:
                cartesian_vel += np.array(key_mapping[k])  # Sum velocities properly

            J = compute_jacobian(joint_angles)
            if J is None:
                continue
            
            try:
                #q_dot = np.dot(np.linalg.inv(J), cartesian_vel)
                lambda_ = 0.01  # Small damping factor
                J_pinv_damped = np.dot(J.T, np.linalg.inv(J @ J.T + lambda_ * np.eye(6)))
                q_dot = np.dot(J_pinv_damped, cartesian_vel)

            except np.linalg.LinAlgError:
                rospy.logerr("Jacobian matrix is singular! Try different input.")
                continue
            
            # Set small velocities to zero to prevent drift
            q_dot[np.abs(q_dot) < 1e-3] = 0

            velocity_msg.data = q_dot.tolist()
            rate.sleep()
        
        joint_velocity_pub.publish(velocity_msg)
        rospy.loginfo("Published Joint Velocities: %s", velocity_msg.data)
        rate.sleep()

if __name__ == "__main__":
    try:
        compute_and_publish()
    except rospy.ROSInterruptException:
        pass
