import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray
from collections import deque

rospy.init_node('aruco_tracker')
position_pub = rospy.Publisher('/aruco_position', Float32MultiArray, queue_size=10)

# End effector's starting position in base frame (from your XML and calibration)
END_EFFECTOR_BASE_POSITION = np.array([0.0003, 0.4984,0.3775])  # x, y, z

# Camera setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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

try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        if ids is not None:
            for corner in corners:
                # Pose estimation
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
                    # Coordinate adjustments
                    shifted_tvec = tvec.flatten() - np.array([0, 0, 00])  # Camera offset
                    x, y, z = shifted_tvec
                    current_position = np.array([x, -y, z])  # Convert to ROS (Y-up)

                    # Store position for smoothing
                    position_buffer.append(current_position)
                    smoothed = np.mean(position_buffer, axis=0)

                    # Handle calibration (key 'c' pressed)
                    key = cv2.waitKey(1)
                    if key == ord('c'):
                        calibration_origin = smoothed.copy()
                        rospy.loginfo(f"Calibration Origin Set: {calibration_origin}")
                        # On calibration, publish the end effector base position so the cube starts there
                        position_msg = Float32MultiArray(data=END_EFFECTOR_BASE_POSITION.tolist())
                        position_pub.publish(position_msg)

                    # After calibration, publish absolute position in base frame
                    if calibration_origin is not None:
                        # Calculate relative motion from calibration point
                        relative_motion = smoothed - calibration_origin
                        # Add to known end effector base position
                        absolute_position = END_EFFECTOR_BASE_POSITION + relative_motion
                        position_msg = Float32MultiArray(data=absolute_position.tolist())
                        position_pub.publish(position_msg)
                        rospy.loginfo(f"End Effector Position: x={absolute_position[0]:.3f}, y={absolute_position[1]:.3f}, z={absolute_position[2]:.3f}")

        # Display
        cv2.imshow("ArUco Tracker (Press 'c' to set origin)", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
