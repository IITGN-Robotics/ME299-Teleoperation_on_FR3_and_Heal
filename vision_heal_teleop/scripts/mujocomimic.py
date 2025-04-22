#this code works fine except there is some error with the orientation part

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import mujoco
import mujoco.viewer
import mink
from scipy.signal import savgol_filter

# ------------------------ Config ------------------------------------
_XML_PATH = "/home/pikapika/Debojit_WS/RL-Based-Dual-Arm-Manipulation/Dual_Arm_Manipulation/robot_descriptions/addverb_heal_scene.xml"
SOLVER='quadprog'
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 10
rate_hz = 500.0
MAX_VEL = 0.1
SMOOTH_ALPHA = 0.2  # For mocap target smoothing

# ------------------------ Initialize MuJoCo & Mink ------------------
model = mujoco.MjModel.from_xml_path(_XML_PATH)
data = mujoco.MjData(model)
configuration = mink.Configuration(model)

end_effector_task = mink.FrameTask("attachment_site", "site", 1.0, 1.0, 1.0)
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

# ------------------------ Main Loop ---------------------------------
prev_vel = np.zeros(NUM_JOINTS)  # Initialize outside loop

with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=True) as viewer:
    rospy.loginfo("Starting interactive IK loop...")

    while viewer.is_running() and not rospy.is_shutdown():
        
        current_target = data.mocap_pos[0].copy()
        filtered_target = SMOOTH_ALPHA * current_target + (1 - SMOOTH_ALPHA) * prev_target
        prev_target = filtered_target.copy()
        data.mocap_pos[0] = filtered_target

        posture_task.set_target_from_configuration(configuration)
        T_wt = mink.SE3.from_mocap_name(model, data, "target")
        end_effector_task.set_target(T_wt)

        for _ in range(MAX_ITERS):
            vel = mink.solve_ik(configuration, tasks, 1.0 / rate_hz, SOLVER, 1e-3)
            err = end_effector_task.compute_error(configuration)
            dist = np.linalg.norm(err[:3])

            # Soft scaling using tanh instead of clipping
            scaled_vel = np.tanh(vel * 5.0) * MAX_VEL

            # Low-pass temporal smoothing
            smoothed_vel = 0.2 * scaled_vel + 0.8 * prev_vel
            prev_vel = smoothed_vel.copy()

            configuration.integrate_inplace(smoothed_vel, 1.0 / rate_hz)
            print("Position error:", np.linalg.norm(err[:3]))
            print("Orientation error:", np.linalg.norm(err[3:]))

            if np.linalg.norm(err[:3]) <= POS_THRESHOLD and np.linalg.norm(err[3:]) <= ORI_THRESHOLD:
                break

        vel_msg = Float64MultiArray()
        vel_msg.data = smoothed_vel.tolist()
        vel_pub.publish(vel_msg)

        time_steps.append(t)
        for i in range(NUM_JOINTS):
            published_velocities[i].append(smoothed_vel[i])

        data.ctrl = configuration.q
        mujoco.mj_step(model, data)
        viewer.sync()
        t += 1
        rate.sleep()


# ------------------------ Post-Execution Plot -----------------------
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



