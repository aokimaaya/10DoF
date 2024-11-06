import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
from screeninfo import get_monitors

class ArUcoMarkerDetector:
    def __init__(self):
        # Get the second monitor
        second_monitor = get_monitors()[0]

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

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.depth_profile = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        self.depth_intrinsics = self.depth_profile.as_video_stream_profile().get_intrinsics()

        # Get the depth scale
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        print(self.depth_scale)

        # ArUco marker dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.unit = 1
        # Marker size (in cm)
        self.marker_size = 0.038 * self.unit

        # Define the marker IDs
        self.middle_marker_id = 4
        self.first_marker_id = 3
        self.highest_marker_id = 7

    def run(self):
        while True:
            # Wait for a new frame from RealSense
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert color image to grayscale
            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

            # Check if at least three markers are detected
            if ids is not None and len(ids) >= 3:
                # Draw circles at corners and centers of each marker
                for i in range(len(corners)):
                    for j in range(4):
                        corner = tuple(corners[i][0][j].astype(int))
                        cv2.circle(color_image, corner, 5, (0, 255, 0), -1)
                    marker_center = np.mean(corners[i][0], axis=0).astype(int)
                    cv2.circle(color_image, tuple(marker_center), 5, (255, 0, 0), -1)

                # Draw lines connecting all the corners of each marker
                for marker_corners in corners:
                    cv2.polylines(color_image, np.int32([marker_corners]), True, (0, 0, 255), 2)

                # Rest of the code...

            # Display the color image
            cv2.imshow("ArUco Marker Detection", color_image)

            # Exit the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Stop the pipeline
        self.pipeline.stop()

        # Close all OpenCV windows
        cv2.destroyAllWindows()

    def calculate_3d_coordinates(self, depth_frame, center):
        x, y = center
        depth_value = depth_frame.get_distance(x, y)
        depth = depth_value * self.depth_scale
        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
        return point


if __name__ == "__main__":
    marker_detector = ArUcoMarkerDetector()
    marker_detector.run()
