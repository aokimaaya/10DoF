import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
from screeninfo import get_monitors

# Get the second monitor
second_monitor = get_monitors()[1]

# Get the width and height of the second monitor
monitor_width = second_monitor.width
monitor_height = second_monitor.height

# Calculate the position for the window
window_x = second_monitor.x 
window_y = second_monitor.y

window_width = int(monitor_width / 2)
window_height = int(monitor_height * 480 / 640)

# Set the window position
cv2.namedWindow('ArUco Marker Detection', cv2.WINDOW_NORMAL)
cv2.moveWindow('ArUco Marker Detection', window_x, window_y)
cv2.resizeWindow('ArUco Marker Detection', window_width, window_height)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# ArUco marker dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Marker size (in cm)
marker_size = 3.8

# Define the marker IDs
middle_marker_id = 10
first_marker_id = 8
highest_marker_id =11

# Loop for capturing frames
while True:
    # Wait for a new frame from RealSense
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # Convert color image to grayscale
    color_image = np.asanyarray(color_frame.get_data())
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

    # Check if at least three markers are detected
    if ids is not None and len(ids) >= 3:
        marker_ids = ids.flatten()

        # Find the indices of the middle, first, and highest markers
        middle_marker_index = np.where(marker_ids == middle_marker_id)[0][0]
        first_marker_index = np.where(marker_ids == first_marker_id)[0][0]
        highest_marker_index = np.where(marker_ids == highest_marker_id)[0][0]

        # Get the corners of the middle, first, and highest markers
        middle_marker_corners = corners[middle_marker_index][0]
        first_marker_corners = corners[first_marker_index][0]
        highest_marker_corners = corners[highest_marker_index][0]

        # Calculate the center points of the markers
        middle_marker_center = np.mean(middle_marker_corners, axis=0).astype(int)
        first_marker_center = np.mean(first_marker_corners, axis=0).astype(int)
        highest_marker_center = np.mean(highest_marker_corners, axis=0).astype(int)

        # Calculate the lengths of the vectors
        vector1 = first_marker_center - middle_marker_center
        vector2 = highest_marker_center - middle_marker_center
        length_vector1 = np.linalg.norm(vector1) * marker_size
        length_vector2 = np.linalg.norm(vector2) * marker_size

        # Calculate the angle between the vectors
        cosine_angle = np.dot(vector1, vector2) / (length_vector1 * length_vector2)
        angle = np.degrees(np.arccos(cosine_angle))

        # Display the lengths and angle
        cv2.putText(color_image, f"Length 1: {length_vector1:.2f} cm", tuple(np.mean([middle_marker_center, first_marker_center], axis=0).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(color_image, f"Length 2: {length_vector2:.2f} cm", tuple(np.mean([middle_marker_center, highest_marker_center], axis=0).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(color_image, f"Angle: {angle:.2f} degrees", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Draw lines connecting the markers
        cv2.line(color_image, tuple(middle_marker_center), tuple(first_marker_center), (0, 0, 255), 2)
        cv2.line(color_image, tuple(middle_marker_center), tuple(highest_marker_center), (0, 0, 255), 2)

        # Draw the marker IDs
        cv2.putText(color_image, f"ID: {middle_marker_id}", tuple(middle_marker_corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(color_image, f"ID: {first_marker_id}", tuple(first_marker_corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(color_image, f"ID: {highest_marker_id}", tuple(highest_marker_corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show the resulting image
    cv2.imshow('ArUco Marker Detection', color_image)

    # Exit loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and close all windows
pipeline.stop()
cv2.destroyAllWindows()
