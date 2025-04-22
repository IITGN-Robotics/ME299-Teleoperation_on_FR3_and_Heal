import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import rospy
import time
from collections import deque
from std_msgs.msg import Float32MultiArray
from addverb_cobot_msgs.msg import GraspActionGoal, ReleaseActionGoal

# ========== ROS NODE INIT ==========
rospy.init_node('aruco_tracker')
position_pub = rospy.Publisher('/aruco_position', Float32MultiArray, queue_size=10)

# End effector's starting position in base frame (from your XML and calibration)
END_EFFECTOR_BASE_POSITION = np.array([0.0003, 0.4984, 0.3775])  # x, y, z

# Camera setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
align = rs.align(rs.stream.color)
pipeline.start(config)

# ArUco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Camera calibration
fx, fy, cx, cy = 615.0, 615.0, 320.0, 240.0
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)
marker_length = 0.05  # 5cm marker

# Calibration variables
calibration_origin = None
position_buffer = deque(maxlen=25)
rate = rospy.Rate(30)

# ========== MEDIAPIPE SETUP ==========
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Gripper calibration variables
calibrated = False
d_px_ref = None
d_cm_ref = 12.0  # Set this to the real distance between fingers during calibration (in cm)

def get_2d_pixel_distance(thumb_uv, index_uv):
    return np.linalg.norm(np.array(thumb_uv) - np.array(index_uv))

# ========== GRIPPER CONTROLLER CLASS ==========
class GripperController:
    def __init__(self):
        self.gripper_closed = False
        self.last_action_time = 0
        self.action_cooldown = 1.0  # seconds
        self.consecutive_above_threshold = 0
        self.consecutive_high_readings = 0
        self.required_consecutive = 5  # Normal open
        self.max_consecutive_high = 5  # High distance
        self.high_distance_threshold = 50  # cm
        self.grasp_pub = rospy.Publisher('/robotA/grasp_action/goal', GraspActionGoal, queue_size=1)
        self.release_pub = rospy.Publisher('/robotA/release_action/goal', ReleaseActionGoal, queue_size=1)
        rospy.sleep(0.5)
        print("Gripper controllers ready.")

    def control_gripper(self, current_dist):
        if current_dist is None:
            return
        if current_dist > self.high_distance_threshold:
            self.consecutive_high_readings += 1
            if self.consecutive_high_readings >= self.max_consecutive_high and self.gripper_closed:
                self._send_release_command()
                self.consecutive_high_readings = 0
                return
        else:
            self.consecutive_high_readings = 0
        THRESHOLD = 3.5  # cm
        if time.time() - self.last_action_time < self.action_cooldown:
            return
        if not self.gripper_closed:
            if current_dist < THRESHOLD:
                self._send_grasp_command()
        else:
            if current_dist >= THRESHOLD:
                self.consecutive_above_threshold += 1
                if self.consecutive_above_threshold >= self.required_consecutive:
                    self._send_release_command()
                    self.consecutive_above_threshold = 0
            else:
                self.consecutive_above_threshold = 0

    def _send_grasp_command(self):
        msg = GraspActionGoal()
        msg.goal.grasp_force = 100
        self.grasp_pub.publish(msg)
        self.gripper_closed = True
        self.last_action_time = time.time()
        rospy.loginfo("Closing gripper")

    def _send_release_command(self):
        msg = ReleaseActionGoal()
        self.release_pub.publish(msg)
        self.gripper_closed = False
        self.last_action_time = time.time()
        rospy.loginfo("Opening gripper")

# ========== MAIN LOOP ==========
gripper_controller = GripperController()

try:
    with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco_detector.detectMarkers(gray)

            # --- ArUco logic ---
            if ids is not None:
                for corner in corners:
                    success, rvec, tvec = cv2.solvePnP(
                        np.array([[-marker_length/2, marker_length/2, 0],
                                  [marker_length/2, marker_length/2, 0],
                                  [marker_length/2, -marker_length/2, 0],
                                  [-marker_length/2, -marker_length/2, 0]], dtype=np.float32),
                        corner,
                        camera_matrix,
                        dist_coeffs,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if success:
                        shifted_tvec = tvec.flatten() - np.array([0, 0, 0])
                        x, y, z = shifted_tvec
                        current_position = np.array([x, -y, z])
                        position_buffer.append(current_position)
                        smoothed = np.mean(position_buffer, axis=0)

                        key = cv2.waitKey(1) & 0xFF
                        # --- Calibration logic for both systems ---
                        if key == ord('c'):
                            calibration_origin = smoothed.copy()
                            rospy.loginfo(f"Calibration Origin Set: {calibration_origin}")
                            position_msg = Float32MultiArray(data=END_EFFECTOR_BASE_POSITION.tolist())
                            position_pub.publish(position_msg)
                            # Also calibrate gripper distance if hand is visible
                            results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                            if results.multi_hand_landmarks:
                                hand_landmarks = results.multi_hand_landmarks[0]
                                thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                                index = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                                h, w = color_image.shape[:2]
                                thumb_uv = (int(thumb.x * w), int(thumb.y * h))
                                index_uv = (int(index.x * w), int(index.y * h))
                                d_px_ref = get_2d_pixel_distance(thumb_uv, index_uv)
                                calibrated = True
                                print(f"Finger distance calibrated: d_px_ref={d_px_ref:.2f} pixels")
                            else:
                                print("Finger distance calibration failed: No hand detected.")

                        if calibration_origin is not None:
                            relative_motion = smoothed - calibration_origin
                            absolute_position = END_EFFECTOR_BASE_POSITION + relative_motion
                            position_msg = Float32MultiArray(data=absolute_position.tolist())
                            position_pub.publish(position_msg)
                            rospy.loginfo(f"End Effector Position: x={absolute_position[0]:.3f}, y={absolute_position[1]:.3f}, z={absolute_position[2]:.3f}")

            # --- Gripper logic ---
            results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            current_distance = None
            if results.multi_hand_landmarks and calibrated and d_px_ref is not None:
                hand_landmarks = results.multi_hand_landmarks[0]
                thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                index = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                h, w = color_image.shape[:2]
                thumb_uv = (int(thumb.x * w), int(thumb.y * h))
                index_uv = (int(index.x * w), int(index.y * h))
                d_px_cur = get_2d_pixel_distance(thumb_uv, index_uv)
                current_distance = d_cm_ref * (d_px_cur / d_px_ref)
                cv2.putText(color_image, f"Distance: {current_distance:.2f} cm", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            elif not calibrated:
                cv2.putText(color_image, "Press 'c' to calibrate", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            # Gripper control
            gripper_controller.control_gripper(current_distance)

            # Display
            cv2.imshow("ArUco Tracker (Press 'c' to set origin and calibrate fingers)", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
