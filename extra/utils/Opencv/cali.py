import os
import cv2
import numpy as np
import pyrealsense2 as rs

def calibrate_depth_sensor():
    # Define the Charuco board parameters
    board_size = (5, 7)  # Size of the board in squares (X, Y)
    square_length = 0.04  # Length of each square edge in meters
    marker_length = 0.03  # Length of the marker edge in meters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((board_size[0], board_size[1]), square_length, marker_length, dictionary)

    # Create the calibration arrays
    allCorners = []
    allIds = []
    imageSize = None

    # Create a context object
    pipeline = rs.pipeline()

    # Configure and start the pipeline with the depth and color streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    pipeline.start(config)

    try:


        while True:
            # Wait for the next frameset
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert the color frame to BGR for OpenCV
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # Detect markers and corners
            markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
                color_image, dictionary
            )
            try:
                _, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(
                    markerCorners, markerIds, color_image, board
                )
            except cv2.error as e:
                print("Error occurred during corner interpolation:", e)
                continue

            # If any Charuco corners are detected, add them to the calibration arrays
            if charucoCorners is not None and charucoIds is not None:
                allCorners.append(charucoCorners)
                allIds.append(charucoIds)
                imageSize = color_image.shape[:2]
                # image_count += 1

            # Draw the detected markers and corners on the color image
            color_image = cv2.aruco.drawDetectedMarkers(
                color_image, markerCorners, markerIds
            )
            if charucoCorners is not None and charucoIds is not None:
                color_image = cv2.aruco.drawDetectedCornersCharuco(
                    color_image, charucoCorners, charucoIds
                )

            # Display the color image with markers and corners
            cv2.imshow("Calibration", color_image)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

            if len(allCorners) > 0:
                # Perform the calibration
                cameraMatrixInit = np.array([[1.0, 0, 0.5 * imageSize[0]],
                                            [0, 1.0, 0.5 * imageSize[1]],
                                            [0, 0, 1.0]], dtype=np.float32)
                distCoeffsInit = np.zeros((5, 1), np.float32)

                try:
                    _, cameraMatrix, distCoeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
                        allCorners,
                        allIds,
                        board,
                        imageSize,
                        cameraMatrixInit,
                        distCoeffsInit
                    )
                except cv2.error as e:
                    print("Error occurred during camera calibration:", e)
                    return

                # Print the calibration matrix and distortion coefficients
                print("Calibration complete:")
                print("Camera Matrix:")
                print(cameraMatrix)
                print("Distortion Coefficients:")
                print(distCoeffs)

                # Get the current file's directory
                script_dir = os.path.dirname(os.path.abspath(__file__))

                # Save the calibration matrix and distortion coefficients to files
                matrix_file_path = os.path.join(script_dir, "camera_matrix_1.txt")
                coeffs_file_path = os.path.join(script_dir, "distortion_coefficients_1.txt")
                try:
                    np.savetxt(matrix_file_path, cameraMatrix)
                    np.savetxt(coeffs_file_path, distCoeffs)
                except Exception as e:
                    print("Error occurred while saving the files:", e)

    finally:
        # Stop the pipeline and release resources
        pipeline.stop()
        cv2.destroyAllWindows()

# Run the calibration function
calibrate_depth_sensor()
