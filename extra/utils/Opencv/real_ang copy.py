import cv2 ,os
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
from screeninfo import get_monitors

class ArUcoMarkerDetector:
    def __init__(self):
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
        self.middle_marker_id = 10
        self.first_marker_id = 8
        self.highest_marker_id = 11

        # Load the camera calibration data
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # self.camera_matrix = np.loadtxt( os.path.join(script_dir, 'camera_matrix.txt'))
        # self.dist_coeffs = np.loadtxt( os.path.join(script_dir, 'distortion_coefficients.txt'))

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
                # Find the indices of the middle, first, and highest markers
                middle_marker_index = np.where(ids.flatten() == self.middle_marker_id)[0][0]
                first_marker_index = np.where(ids.flatten() == self.first_marker_id)[0][0]
                highest_marker_index = np.where(ids.flatten() == self.highest_marker_id)[0][0]

                # Get the corners of the middle, first, and highest markers
                middle_marker_corners = corners[middle_marker_index][0]
                first_marker_corners = corners[first_marker_index][0]
                highest_marker_corners = corners[highest_marker_index][0]

                # Calculate the 2D coordinates of the marker centers
                middle_marker_center = np.mean(middle_marker_corners, axis=0).astype(int)
                first_marker_center = np.mean(first_marker_corners, axis=0).astype(int)
                highest_marker_center = np.mean(highest_marker_corners, axis=0).astype(int)

                # Calculate the 3D coordinates of the markers
                middle_marker_coordinates = self.calculate_3d_coordinates(depth_frame, middle_marker_center)
                first_marker_coordinates = self.calculate_3d_coordinates(depth_frame, first_marker_center)
                highest_marker_coordinates = self.calculate_3d_coordinates(depth_frame, highest_marker_center)

                # Convert the coordinate lists to NumPy arrays
                middle_marker_coordinates = np.array(middle_marker_coordinates)
                first_marker_coordinates = np.array(first_marker_coordinates)
                highest_marker_coordinates = np.array(highest_marker_coordinates)

                # Calculate the lengths and angle using 3D coordinates
                vector1 = first_marker_coordinates - middle_marker_coordinates
                vector2 = highest_marker_coordinates - middle_marker_coordinates
                length_vector1 = np.sqrt(np.sum(vector1**2))
                length_vector2 = np.sqrt(np.sum(vector2**2))
                cosine_angle = np.dot(vector1, vector2) / (length_vector1 * length_vector2)
                angle = np.degrees(np.arccos(cosine_angle))

                # Rest of the code...

                # Display the lengths and angle
                cv2.putText(color_image, f"Length 1: {length_vector1:.5f} m", tuple(np.mean([middle_marker_center, first_marker_center], axis=0, dtype=int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(color_image, f"Length 2: {length_vector2:.5f} m", tuple(np.mean([middle_marker_center, highest_marker_center], axis=0, dtype=int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(color_image, f"Angle: {angle:.3f} degrees", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Draw lines connecting the markers
                cv2.line(color_image, tuple(middle_marker_center), tuple(first_marker_center), (0, 0, 255), 2)
                cv2.line(color_image, tuple(middle_marker_center), tuple(highest_marker_center), (0, 0, 255), 2)

                # Draw the marker IDs
                cv2.putText(color_image, f"ID: {self.middle_marker_id}", tuple(map(int, middle_marker_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(color_image, f"ID: {self.first_marker_id}", tuple(map(int, first_marker_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(color_image, f"ID: {self.highest_marker_id}", tuple(map(int, highest_marker_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

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
