import cv2
import numpy as np
import pyrealsense2 as rs
import os


class aru_ang:
    def __init__(self):
        self.pipeline = None
        self.aruco_dict = None
        self.parameters = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.depth_intrinsics = None
        self.depth_scale = None
        self.angle = None

    def initialize_pipeline(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device('141322250166')  # Replace 'DEVICE_SERIAL_NUMBER_1' with the actual serial number of the D455 camera
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.depth_profile = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        self.depth_intrinsics = self.depth_profile.as_video_stream_profile().get_intrinsics()
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        print(self.depth_scale)

    def initialize_aruco(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.unit = 1
        self.marker_size = 0.089 * self.unit
        self.middle_marker_id = 2
        self.first_marker_id = 1
        self.highest_marker_id = 3

    def load_calibration_data(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.camera_matrix = None  # np.loadtxt( os.path.join(script_dir, 'camera_matrix_1.txt'))
        self.dist_coeffs = None  # np.loadtxt( os.path.join(script_dir, 'distortion_coefficients_1.txt'))

    def run(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=10000)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)
        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

        if ids is not None and len(ids) >= 3:
            middle_marker_index = np.where(ids.flatten() == self.middle_marker_id)[0]
            first_marker_index = np.where(ids.flatten() == self.first_marker_id)[0]
            highest_marker_index = np.where(ids.flatten() == self.highest_marker_id)[0]

            if len(middle_marker_index) > 0 and len(first_marker_index) > 0 and len(highest_marker_index) > 0:
                middle_marker_index = middle_marker_index[0]
                first_marker_index = first_marker_index[0]
                highest_marker_index = highest_marker_index[0]

                middle_marker_corners = corners[middle_marker_index][0]
                first_marker_corners = corners[first_marker_index][0]
                highest_marker_corners = corners[highest_marker_index][0]

                middle_marker_corner = middle_marker_corners[3]
                first_marker_corner = first_marker_corners[3]
                highest_marker_corner = highest_marker_corners[3]

                middle_marker_coordinates = self.calculate_3d_coordinates(depth_frame, middle_marker_corner)
                first_marker_coordinates = self.calculate_3d_coordinates(depth_frame, first_marker_corner)
                highest_marker_coordinates = self.calculate_3d_coordinates(depth_frame, highest_marker_corner)

                middle_marker_corner = tuple(map(int, middle_marker_corner))
                first_marker_corner = tuple(map(int, first_marker_corner))
                highest_marker_corner = tuple(map(int, highest_marker_corner))

                cv2.line(color_image, middle_marker_corner, first_marker_corner, (0, 0, 255), 2)
                cv2.line(color_image, middle_marker_corner, highest_marker_corner, (0, 0, 255), 2)

                return color_image, middle_marker_corner, first_marker_corner, highest_marker_corner

        return None

    def calculate_3d_coordinates(self, depth_frame, center):
        x, y = center
        depth_value = depth_frame.get_distance(x, y)
        depth = depth_value * self.depth_scale * self.unit
        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
        return point


    def calculate_angle(self,color_image, middle_marker_corner, first_marker_corner, highest_marker_corner):
        middle_x, middle_y = middle_marker_corner
        first_x, first_y = first_marker_corner
        highest_x, highest_y = highest_marker_corner

        # Calculate vectors
        vector1 = np.array([first_x - middle_x, first_y - middle_y])
        vector2 = np.array([highest_x - middle_x, highest_y - middle_y])

        # Calculate the angle between the vectors
        dot_product = np.dot(vector1, vector2)
        magnitude1 = np.linalg.norm(vector1)
        magnitude2 = np.linalg.norm(vector2)
        cos_theta = dot_product / (magnitude1 * magnitude2)
        angle = np.arccos(cos_theta)
        angle_degrees = np.degrees(angle)

        return angle_degrees


if __name__ == "__main__":
    marker_detector = aru_ang()
    marker_detector.initialize_pipeline()
    marker_detector.initialize_aruco()
    marker_detector.load_calibration_data()

    while True:
        result = marker_detector.run()

        if result is not None:
            color_image, middle_marker_corner, first_marker_corner, highest_marker_corner = result
            angle = marker_detector.calculate_angle(color_image, middle_marker_corner, first_marker_corner, highest_marker_corner)
            cv2.putText(color_image, f"Angle: {angle:.3f} degrees", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 200, 0), 3)

            cv2.imshow("ArUco Marker Detection", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    marker_detector.pipeline.stop()
    cv2.destroyAllWindows()
