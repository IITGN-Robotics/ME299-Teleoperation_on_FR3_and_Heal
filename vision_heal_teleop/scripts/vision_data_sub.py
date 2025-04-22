#!/usr/bin/env python3

from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from loop_rate_limiters import RateLimiter
import mink
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from scipy.signal import savgol_filter
from collections import deque



# ------------------------ Config ------------------------------------
_XML_PATH = "/home/pikapika/Debojit_WS/RL-Based-Dual-Arm-Manipulation/Dual_Arm_Manipulation/robot_descriptions/addverb_heal_scene.xml"
SOLVER='quadprog'
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 3
rate_hz = 500.0
MAX_VEL = 0.1
SMOOTH_ALPHA = 0.2  # For mocap target smoothing

# Global to store ArUco marker position
marker_pos = None
VEL_BUFFER_SIZE = 5
vel_buffer = deque(maxlen=VEL_BUFFER_SIZE)

# -------------------- CALLBACK ---------------------
def marker_callback(msg):
    global marker_pos
    # Swap Y and Z axes
    x, y, z = msg.data
    marker_pos = np.array([z, -x, y])


# ------------------------ Initialize MuJoCo & Mink ------------------
model = mujoco.MjModel.from_xml_path(_XML_PATH)
data = mujoco.MjData(model)
configuration = mink.Configuration(model)

end_effector_task = mink.FrameTask("attachment_site", "site", 0.5, 0.5, 1.0)
posture_task = mink.PostureTask(model, 1e-2)
tasks = [end_effector_task, posture_task]

NUM_JOINTS = model.nq
latest_joint_states = None

# ------------------------ ROS Setup ---------------------------------
def joint_state_callback(msg):
    global latest_joint_states
    latest_joint_states = msg

rospy.init_node('ik_velocity_commander')
vel_pub = rospy.Publisher('/velocity_controller/command', Float64MultiArray, queue_size=1)
joint_sub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
rate = rospy.Rate(rate_hz)

# rospy.init_node("ik_aruco_follower")
rospy.Subscriber("/aruco_position", Float32MultiArray, marker_callback)

# ------------------------ Joint Init --------------------------------
rospy.loginfo("Waiting for /joint_states...")
try:
    init_joint_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
except rospy.ROSException:
    rospy.logwarn("No joint_states. Resetting to home.")
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    configuration.update(data.qpos)
    mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")
else:
    for i in range(min(NUM_JOINTS, len(init_joint_msg.position))):
        data.qpos[i] = init_joint_msg.position[i]
    mujoco.mj_forward(model, data)
    configuration.update(data.qpos)
    mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")

posture_task.set_target_from_configuration(configuration)

rate = RateLimiter(frequency=500.0, warn=False)
mujoco.mjv_defaultFreeCamera(model, mujoco.MjvCamera())

# ------------------------ Motion Setup -----------------------------
original_target = data.mocap_pos[0].copy()
prev_target = original_target.copy()

# ------------------------ Logging Setup -----------------------------
published_velocities = [[] for _ in range(NUM_JOINTS)]
time_steps = []
t = 0

# ------------------------ Shutdown Handler --------------------------
def stop_robot():
    stop_msg = Float64MultiArray()
    stop_msg.data = [0.0] * NUM_JOINTS
    vel_pub.publish(stop_msg)
    rospy.loginfo("Robot stopped.")

rospy.on_shutdown(stop_robot)


# Logging setup for joint positions
sim_joint_positions = [[] for _ in range(NUM_JOINTS)]   # MuJoCo simulated joint angles
real_joint_positions = [[] for _ in range(NUM_JOINTS)]  # Real robot joint angles

# -------------------- Main Loop ---------------------------
prev_vel = np.zeros(NUM_JOINTS)  # Initialize outside loop

with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=True) as viewer:
    rospy.loginfo("IK ArUco follower started...")

    while viewer.is_running() and not rospy.is_shutdown():
        # Move the cube based on marker position
        if marker_pos is not None:
            # Update mocap target site with the marker position
            data.mocap_pos[0] = marker_pos

        # Smooth target position for real robot
        current_target = data.mocap_pos[0].copy()
        filtered_target = SMOOTH_ALPHA * current_target + (1 - SMOOTH_ALPHA) * prev_target
        prev_target = filtered_target.copy()
        data.mocap_pos[0] = filtered_target

        posture_task.set_target_from_configuration(configuration)
        T_wt = mink.SE3.from_mocap_name(model, data, "target")
        end_effector_task.set_target(T_wt)

        # Solve IK to match end-effector to target
        for _ in range(MAX_ITERS):
            vel = mink.solve_ik(configuration, tasks, 1.0 / rate_hz, SOLVER, 1e-3)
            err = end_effector_task.compute_error(configuration)
            dist = np.linalg.norm(err[:3])

            # Soft scaling using tanh instead of clipping
            scaled_vel = np.tanh(vel * 5.0) * MAX_VEL

            # Low-pass temporal smoothing
            # Append new velocity
            vel_buffer.append(scaled_vel)

            # Average over buffer (if full, average all)
            if len(vel_buffer) > 0:
                smoothed_vel = np.mean(vel_buffer, axis=0)
            else:
                smoothed_vel = scaled_vel

            prev_vel = smoothed_vel.copy()
            ee_pos = data.site_xpos[model.site("attachment_site").id]
            print(f"End-Effector Position: x={ee_pos[0]:.3f}, y={ee_pos[1]:.3f}, z={ee_pos[2]:.3f}")


            configuration.integrate_inplace(smoothed_vel, 1.0 / rate_hz)
            print("Position error:", np.linalg.norm(err[:3]))
            print("Orientation error:", np.linalg.norm(err[3:]))

            if np.linalg.norm(err[:3]) <= POS_THRESHOLD and np.linalg.norm(err[3:]) <= ORI_THRESHOLD:
                break

        # Publish velocity for real robot
                # Force last joint velocity to zero
        smoothed_vel[-1] = 0.0

        # Publish velocity for real robot
        vel_msg = Float64MultiArray()
        vel_msg.data = smoothed_vel.tolist()
        vel_pub.publish(vel_msg)


        # Logging the joint velocities
        time_steps.append(t)
        for i in range(NUM_JOINTS):
            published_velocities[i].append(smoothed_vel[i])

        # Store MuJoCo simulated joint positions
        for i in range(NUM_JOINTS):
            sim_joint_positions[i].append(configuration.q[i])

        # Store real robot joint positions (if available)
        if latest_joint_states is not None and len(latest_joint_states.position) >= NUM_JOINTS:
            for i in range(NUM_JOINTS):
                real_joint_positions[i].append(latest_joint_states.position[i])
        else:
            for i in range(NUM_JOINTS):
                real_joint_positions[i].append(None)  # Mark missing data


        # Apply updated joint angles
        data.ctrl = configuration.q
        mujoco.mj_step(model, data)
        viewer.sync()
        t += 1
        rate.sleep()


# -------------------- Post-Execution Plot --------------------
fig, axs = plt.subplots(NUM_JOINTS, 1, figsize=(8, 12))
for i in range(NUM_JOINTS):
    if len(published_velocities[i]) > 7:
        smoothed = savgol_filter(published_velocities[i], 7, 2)
    else:
        smoothed = published_velocities[i]
    axs[i].plot(time_steps, smoothed, label='Smoothed Velocities', color='green')
    axs[i].legend()
    axs[i].set_ylabel(f'Joint {i+1}')
axs[-1].set_xlabel("Time step")
plt.tight_layout()
plt.show()

rospy.loginfo("Execution complete.")

# -------------------- Plot Joint Positions --------------------
fig, axs = plt.subplots(NUM_JOINTS, 1, figsize=(10, 14))
for i in range(NUM_JOINTS):
    sim_joint_plot = sim_joint_positions[i]
    real_joint_plot = real_joint_positions[i]

    axs[i].plot(time_steps, sim_joint_plot, label='Simulated (MuJoCo)', color='blue')
    
    # Filter out None values for the real robot
    if any(p is not None for p in real_joint_plot):
        valid_times = [time_steps[j] for j in range(len(real_joint_plot)) if real_joint_plot[j] is not None]
        valid_vals = [real_joint_plot[j] for j in range(len(real_joint_plot)) if real_joint_plot[j] is not None]
        axs[i].plot(valid_times, valid_vals, label='Real Robot', color='orange', linestyle='--')
    
    axs[i].legend()
    axs[i].set_ylabel(f'Joint {i+1} (rad)')

axs[-1].set_xlabel("Time step")
plt.suptitle("Joint Angles: MuJoCo vs Real Robot", fontsize=14)
plt.tight_layout()
plt.show()