import cv2
import numpy as np
import pyrealsense2 as rs
from cv2 import aruco
from screeninfo import get_monitors

# Define ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Define ArUco parameters
aruco_params = aruco.DetectorParameters()

# Initialize the RealSense pipeline for camera 1 (D435)
pipeline1 = rs.pipeline()
config1 = rs.config()
config1.enable_device('216322072627')  # Replace 'DEVICE_SERIAL_NUMBER_1' with the actual serial number of the D435 camera
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline for camera 1
pipeline1.start(config1)

# Initialize the RealSense pipeline for camera 2 (D455)
pipeline2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('141322250166')  # Replace 'DEVICE_SERIAL_NUMBER_2' with the actual serial number of the D455 camera
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline for camera 2
pipeline2.start(config2)

# Get extended monitor information
monitors = get_monitors()
extended_monitor = monitors[1]  # Assuming the extended monitor is the second monitor

# Calculate window positions
window_width = int(extended_monitor.width / 2)
window_height = int(extended_monitor.height / 2)
window_x1 = extended_monitor.x
window_y1 = extended_monitor.y
window_x2 = window_x1 + window_width
window_y2 = window_y1 + window_height

# Create OpenCV windows
cv2.namedWindow("Camera 1 - ArUco Marker Detection")
cv2.moveWindow("Camera 1 - ArUco Marker Detection", window_x1, window_y1)
cv2.namedWindow("Camera 2 - ArUco Marker Detection")
cv2.moveWindow("Camera 2 - ArUco Marker Detection", window_x2, window_y1)

# Adaptive thresholding parameters
adaptive_threshold_block_size = 11
adaptive_threshold_constant = 2

try:
    while True:
        # Wait for the next frames from the cameras
        frames1 = pipeline1.wait_for_frames()
        color_frame1 = frames1.get_color_frame()

        frames2 = pipeline2.wait_for_frames()
        color_frame2 = frames2.get_color_frame()

        if not color_frame1 or not color_frame2:
            continue

        # Convert the frames to numpy arrays
        color_image1 = np.asanyarray(color_frame1.get_data())
        color_image2 = np.asanyarray(color_frame2.get_data())

        # Convert the images to grayscale
        gray_image1 = cv2.cvtColor(color_image1, cv2.COLOR_BGR2GRAY)
        gray_image2 = cv2.cvtColor(color_image2, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding to grayscale images
        gray_image1_threshold = cv2.adaptiveThreshold(gray_image1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, adaptive_threshold_block_size, adaptive_threshold_constant)
        gray_image2_threshold = cv2.adaptiveThreshold(gray_image2, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, adaptive_threshold_block_size, adaptive_threshold_constant)

        # Detect ArUco markers for camera 1 (D435)
        corners1, ids1, _ = aruco.detectMarkers(gray_image1_threshold, aruco_dict, parameters=aruco_params)

        if ids1 is not None:
            # Draw bounding boxes and IDs on the detected markers for camera 1
            aruco.drawDetectedMarkers(color_image1, corners1, ids1)

        # Detect ArUco markers for camera 2 (D455)
        corners2, ids2, _ = aruco.detectMarkers(gray_image2_threshold, aruco_dict, parameters=aruco_params)

        if ids2 is not None:
            # Draw bounding boxes and IDs on the detected markers for camera 2
            aruco.drawDetectedMarkers(color_image2, corners2, ids2)

        # Display the images from both cameras
        cv2.imshow("Camera 1 - ArUco Marker Detection", color_image1)
        cv2.imshow("Camera 2 - ArUco Marker Detection", color_image2)

        # Move windows to fit the extended monitor
        cv2.moveWindow("Camera 1 - ArUco Marker Detection", window_x1, window_y1)
        cv2.moveWindow("Camera 2 - ArUco Marker Detection", window_x2, window_y1)

        # Check for the 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipelines and close all windows
    pipeline1.stop()
    pipeline2.stop()
    cv2.destroyAllWindows()
