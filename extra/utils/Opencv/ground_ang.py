import cv2
import os
import numpy as np
import pyrealsense2 as rs
import math

class ArUcoMarkerDetector:
    def __init__(self):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device('141322250166')  # Replace 'DEVICE_SERIAL_NUMBER_1' with the actual serial number of the D455 camera

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        
        self.pipeline.start(self.config)
        
        self.depth_profile = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        self.depth_intrinsics = self.depth_profile.as_video_stream_profile().get_intrinsics()

        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.unit = 1
        self.marker_size = 0.038 * self.unit

        self.middle_marker_id = 20
        self.first_marker_id = 19
        self.highest_marker_id = 21

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.camera_matrix = None  # np.loadtxt( os.path.join(script_dir, 'camera_matrix_1.txt'))
        self.dist_coeffs = None  # np.loadtxt( os.path.join(script_dir, 'distortion_coefficients_1.txt'))

    def run(self):
        while True:
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            
            if not depth_frame or not color_frame or not accel_frame or not gyro_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

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

                    middle_marker_center = np.mean(middle_marker_corners, axis=0).astype(int)
                    first_marker_center = np.mean(first_marker_corners, axis=0).astype(int)
                    highest_marker_center = np.mean(highest_marker_corners, axis=0).astype(int)

                    cv2.circle(color_image, tuple(middle_marker_center), 5, (0, 0, 255), -1)
                    cv2.circle(color_image, tuple(first_marker_center), 5, (0, 255, 0), -1)
                    cv2.circle(color_image, tuple(highest_marker_center), 5, (255, 0, 0), -1)

                    cv2.line(color_image, tuple(middle_marker_center), tuple(first_marker_center), (0, 255, 0), 2)
                    cv2.line(color_image, tuple(middle_marker_center), tuple(highest_marker_center), (255, 0, 0), 2)

                    # Calculate vector1 using the marker positions
                    vector1 = first_marker_center - middle_marker_center
                    vector1_length = np.linalg.norm(vector1)
                    vector1_normalized = vector1 / vector1_length

                    # Retrieve IMU data
                    accel_data = accel_frame.as_motion_frame().get_motion_data()
                    gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                    accel_x, accel_y, accel_z = accel_data.x, accel_data.y, accel_data.z
                    gyro_x, gyro_y, gyro_z = gyro_data.x, gyro_data.y, gyro_data.z

                    # Calculate the angle between vector1 and the IMU data
                    angle_with_imu = self.calculate_angle(vector1_normalized, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

                    print("Angle between vector1 and IMU data:", angle_with_imu)

            cv2.imshow("Color Image", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.pipeline.stop()
        cv2.destroyAllWindows()

    def calculate_angle(self, vector1, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        # Calculate the angle between vector1 and the IMU data
        # Convert accelerometer data to roll and pitch angles
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))

        # Convert gyro data to yaw angle
        dt = 0.01  # Time interval between readings (change according to your IMU's update rate)
        yaw = gyro_z * dt

        # Combine roll, pitch, and yaw angles
        vector2 = [math.cos(roll) * math.cos(pitch), math.sin(roll) * math.cos(pitch), math.sin(pitch)]
        angle = math.acos(np.dot(vector1, vector2))

        return math.degrees(angle)

if __name__ == "__main__":
    marker_detector = ArUcoMarkerDetector()
    marker_detector.run()
