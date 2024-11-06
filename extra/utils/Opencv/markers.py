import cv2
import numpy as np
import pyrealsense2 as rs
from cv2 import aruco

# Rest of the code remains the same...


# Define ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Define ArUco parameters
aruco_params = aruco.DetectorParameters()

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for the next frame from the camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert the frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # Draw bounding boxes and IDs on the detected markers
            aruco.drawDetectedMarkers(color_image, corners, ids)
            # Display the IDs of the detected markers
            # if ids is not None:
            #     for i in range(len(ids)):
            #         marker_center = corners[i][0].mean(axis=0).astype(int)
            #         cv2.putText(color_image, str(ids[i][0]), (marker_center[0], marker_center[1] - 10),
            #                     cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

        # Display the image
        cv2.imshow("ArUco Marker Detection", color_image)

        # Check for the 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close all windows
    pipeline.stop()
    cv2.destroyAllWindows()
