import cv2
import os
import numpy as np
import pyrealsense2 as rs
from screeninfo import get_monitors
import random


class ArUcoMarkerDetector:
    def __init__(self, device_serial_number):
        self.device_serial_number = device_serial_number
        self.pipeline = None
        self.config = None
        self.depth_profile = None
        self.depth_intrinsics = None
        self.depth_scale = None
        self.aruco_dict = None
        self.parameters = None
        self.unit = 1
        self.marker_size = 0.089 * self.unit
        self.middle_marker_id = 7
        self.first_marker_id = 5
        self.highest_marker_id = 6
        self.camera_matrix = None
        self.dist_coeffs = None

    def initialize_pipeline(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device_serial_number)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.depth_profile = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        self.depth_intrinsics = self.depth_profile.as_video_stream_profile().get_intrinsics()
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        print(self.depth_scale)

    def load_camera_calibration(self, camera_matrix_file, distortion_coefficients_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.camera_matrix = np.loadtxt(os.path.join(script_dir, camera_matrix_file))
        self.dist_coeffs = np.loadtxt(os.path.join(script_dir, distortion_coefficients_file))

    def detect_markers(self):
        detected_markers = []
        
        while True:
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

            if ids is not None and len(ids) >= 3:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_corners = corners[i][0]
                    marker_center = marker_corners[2]
                    marker_coordinates = self.calculate_3d_coordinates(depth_frame, marker_center)

                    detected_markers.append({
                        'id': marker_id,
                        'coordinates': marker_coordinates
                    })

            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            cv2.imshow("ArUco Marker Detection", color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        return detected_markers

    def calculate_3d_coordinates(self, depth_frame, center):
        x, y = center
        depth_value = depth_frame.get_distance(x, y)
        depth = depth_value * self.depth_scale * self.unit
        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
        return point

    def run(self, camera_matrix_file, distortion_coefficients_file):
        self.initialize_pipeline()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.load_camera_calibration(camera_matrix_file, distortion_coefficients_file)
        markers = self.detect_markers()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        
        return markers


if __name__ == "__main__":
    device_serial_number = '141322250166'  # Replace with the actual serial number of the D455 camera
    camera_matrix_file = 'camera_matrix_1.txt'
    distortion_coefficients_file = 'distortion_coefficients_1.txt'

    marker_detector = ArUcoMarkerDetector(device_serial_number)
    detected_markers = marker_detector.run(camera_matrix_file, distortion_coefficients_file)

    for marker in detected_markers:
        print(f"Marker ID: {marker['id']}")
        print(f"3D Coordinates: {marker['coordinates']}")
        print()
