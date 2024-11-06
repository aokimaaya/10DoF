import os

import cv2
import numpy as np
import pyrealsense2 as rs


class ArUcoMarkerDetector:
    def __init__(self):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(
            "141322250166"
        )  # Replace 'DEVICE_SERIAL_NUMBER_1' with the actual serial number of the D455 camera

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.depth_profile = self.pipeline.get_active_profile().get_stream(
            rs.stream.depth
        )
        self.depth_intrinsics = (
            self.depth_profile.as_video_stream_profile().get_intrinsics()
        )

        # Get the depth scale
        self.depth_scale = (
            self.pipeline.get_active_profile()
            .get_device()
            .first_depth_sensor()
            .get_depth_scale()
        )

        # ArUco marker dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.unit = 1
        # Marker size (in m)
        self.marker_size = 0.089 * self.unit

        # Define the marker IDs
        self.middle_marker_id = 7
        self.first_marker_id = 5
        self.highest_marker_id = 6
        self.end_effector_marker_id = 1
        self.elbow_marker_id = 2
        # Load the camera calibration data
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.camera_matrix = None
        self.dist_coeffs = None

    def run(self):
        while True:
            # Wait for a new frame from RealSense
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert color image to grayscale
            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_image, self.aruco_dict, parameters=self.parameters
            )

            # Check if markers are detected
            if ids is not None and len(ids) >= 3:
                # Find the indices of the middle, first, highest, end-effector, and elbow markers
                middle_marker_index = np.where(ids.flatten() == self.middle_marker_id)[
                    0
                ]
                first_marker_index = np.where(ids.flatten() == self.first_marker_id)[0]
                highest_marker_index = np.where(
                    ids.flatten() == self.highest_marker_id
                )[0]
                end_effector_index = np.where(
                    ids.flatten() == self.end_effector_marker_id
                )[0]
                elbow_index = np.where(ids.flatten() == self.elbow_marker_id)[0]

                # Ensure that all markers are found
                if (
                    len(middle_marker_index) > 0
                    and len(first_marker_index) > 0
                    and len(highest_marker_index) > 0
                    and len(end_effector_index) > 0
                    and len(elbow_index) > 0
                ):
                    middle_marker_index = middle_marker_index[0]
                    first_marker_index = first_marker_index[0]
                    highest_marker_index = highest_marker_index[0]

                    # Get the corners of the middle, first, highest, end-effector, and elbow markers
                    middle_marker_corners = corners[middle_marker_index][0]
                    first_marker_corners = corners[first_marker_index][0]
                    highest_marker_corners = corners[highest_marker_index][0]

                    # Calculate the 3D coordinates of the markers using the selected corners
                    middle_marker_corner = middle_marker_corners[2]
                    first_marker_corner = first_marker_corners[2]
                    highest_marker_corner = highest_marker_corners[2]
                    end_effector_center = np.mean(
                        corners[end_effector_index][0], axis=0
                    ).astype(int)
                    elbow_center = np.mean(corners[elbow_index][0], axis=0).astype(int)

                    # Calculate the 3D coordinates of the markers
                    middle_marker_coordinates = self.calculate_3d_coordinates(
                        depth_frame, middle_marker_corner
                    )
                    first_marker_coordinates = self.calculate_3d_coordinates(
                        depth_frame, first_marker_corner
                    )
                    highest_marker_coordinates = self.calculate_3d_coordinates(
                        depth_frame, highest_marker_corner
                    )
                    end_effector_coordinates = self.calculate_3d_coordinates(
                        depth_frame, end_effector_center
                    )
                    elbow_coordinates = self.calculate_3d_coordinates(
                        depth_frame, elbow_center
                    )

                    # Update Kalman filters with the new measurements
                    middle_marker_filtered = self.update_kalman_filter(
                        self.middle_marker_kalman, middle_marker_coordinates
                    )
                    first_marker_filtered = self.update_kalman_filter(
                        self.first_marker_kalman, first_marker_coordinates
                    )
                    highest_marker_filtered = self.update_kalman_filter(
                        self.highest_marker_kalman, highest_marker_coordinates
                    )
                    end_effector_filtered = self.update_kalman_filter(
                        self.end_effector_kalman, end_effector_coordinates
                    )
                    elbow_filtered = self.update_kalman_filter(
                        self.elbow_kalman, elbow_coordinates
                    )

                    # Convert the filtered coordinates to NumPy arrays
                    middle_marker_coordinates = np.array(middle_marker_filtered)
                    first_marker_coordinates = np.array(first_marker_filtered)
                    highest_marker_coordinates = np.array(highest_marker_filtered)
                    end_effector_coordinates = np.array(end_effector_filtered)
                    elbow_coordinates = np.array(elbow_filtered)

                    # Convert the coordinate lists to NumPy arrays
                    middle_marker_coordinates = np.array(middle_marker_coordinates)
                    first_marker_coordinates = np.array(first_marker_coordinates)
                    highest_marker_coordinates = np.array(highest_marker_coordinates)

                    # Calculate the lengths and angle using 3D coordinates
                    vector1 = first_marker_coordinates - middle_marker_coordinates
                    vector2 = highest_marker_coordinates - middle_marker_coordinates
                    length_vector1 = np.sqrt(np.sum(vector1**2))
                    length_vector2 = np.sqrt(np.sum(vector2**2))
                    cosine_angle = np.dot(vector1, vector2) / (
                        length_vector1 * length_vector2
                    )
                    angle = np.degrees(np.arccos(cosine_angle))

                    # Draw lines connecting the markers
                    middle_marker_corner = tuple(map(int, middle_marker_corner))
                    first_marker_corner = tuple(map(int, first_marker_corner))
                    highest_marker_corner = tuple(map(int, highest_marker_corner))

                    cv2.line(
                        color_image,
                        middle_marker_corner,
                        first_marker_corner,
                        (0, 0, 255),
                        2,
                    )
                    cv2.line(
                        color_image,
                        middle_marker_corner,
                        highest_marker_corner,
                        (0, 0, 255),
                        2,
                    )

                    # Display the color image
                    cv2.imshow("ArUco Marker Detection", color_image)

            # Exit the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Stop the pipeline
        self.pipeline.stop()

        # Close all OpenCV windows
        cv2.destroyAllWindows()

        return (
            middle_marker_coordinates,
            first_marker_coordinates,
            highest_marker_coordinates,
            end_effector_coordinates,
            elbow_coordinates,
            angle,
        )

    def calculate_3d_coordinates(self, depth_frame, center):
        x, y = center
        depth_value = depth_frame.get_distance(x, y)
        depth = depth_value * self.depth_scale * self.unit
        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
        return point


if __name__ == "__main__":
    marker_detector = ArUcoMarkerDetector()
    result = marker_detector.run()
    if result is not None:
        (
            middle_marker_coordinates,
            first_marker_coordinates,
            highest_marker_coordinates,
            end_effector_coordinates,
            elbow_coordinates,
            angle,
        ) = result
        print("Middle Marker 3D Coordinates:", middle_marker_coordinates)
        print("First Marker 3D Coordinates:", first_marker_coordinates)
        print("Highest Marker 3D Coordinates:", highest_marker_coordinates)
        print(
            "Angle between First Marker and Highest Marker with respect to Middle Marker:",
            angle,
        )
        print("End-Effector Marker 3D Coordinates:", end_effector_coordinates)
        print("Elbow Marker 3D Coordinates:", elbow_coordinates)
