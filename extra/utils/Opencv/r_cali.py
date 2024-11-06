import os
import cv2
import numpy as np
import pyrealsense2 as rs
from datetime import datetime

def read_chessboards(images, board):
    """
    Charuco base pose estimation.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
        
        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner, 
                                 winSize=(20, 20), 
                                 zeroZone=(-1, -1), 
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)        
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])              
        
        decimator += 1   

    imsize = gray.shape
    return allCorners, allIds, imsize


# def calibrate_depth_sensor_folder(images_folder):
#     # Define the Charuco board parameters
#     board_size = (5, 7)  # Size of the board in squares (X, Y)
#     square_length = 0.032  # Length of each square edge in meters
#     marker_length = 0.025  # Length of the marker edge in meters
#     dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#     board = cv2.aruco.CharucoBoard_create(board_size[0], board_size[1], square_length, marker_length, dictionary)

#     # Read the images in the folder
#     image_files = os.listdir(images_folder)
#     color_images = []
#     for image_file in image_files:
#         if image_file.endswith(".jpg") or image_file.endswith(".png"):
#             image_path = os.path.join(images_folder, image_file)
#             color_images.append(image_path)

#     if len(color_images) > 0:
#         # Read the Charuco board images and obtain corner coordinates and marker IDs
#         allCorners, allIds, imsize = read_chessboards(color_images, board)

#         if len(allCorners) > 0:
#             # Perform the calibration
#             cameraMatrixInit = np.array([[1.0, 0, 0.5 * imsize[1]],
#                                          [0, 1.0, 0.5 * imsize[0]],
#                                          [0, 0, 1.0]], dtype=np.float32)
#             distCoeffsInit = np.zeros((5, 1), np.float32)

#             try:
#                 _, cameraMatrix, distCoeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
#                     allCorners,
#                     allIds,
#                     board,
#                     imsize,
#                     cameraMatrixInit,
#                     distCoeffsInit
#                 )
#             except cv2.error as e:
#                 print("Error occurred during camera calibration:", e)
#                 return

#             # Print the calibration matrix and distortion coefficients
#             print("Calibration complete:")
#             print("Camera Matrix:")
#             print(cameraMatrix)
#             print("Distortion Coefficients:")
#             print(distCoeffs)

#             # Get the current file's directory
#             script_dir = os.path.dirname(os.path.abspath(__file__))

#             # Save the calibration matrix and distortion coefficients to files
#             matrix_file_path = os.path.join(script_dir, "camera_matrix_1.txt")
#             coeffs_file_path = os.path.join(script_dir, "distortion_coefficients_1.txt")
#             try:
#                 np.savetxt(matrix_file_path, cameraMatrix)
#                 np.savetxt(coeffs_file_path, distCoeffs)
#             except Exception as e:
#                 print("Error occurred while saving the files:", e)
#         else:
#             print("No corners detected in the provided images.")
#     else:
#         print("No images found in the folder.")


def calibrate_depth_sensor(allCorners, allIds, imsize,save_dir):
    # Define the Charuco board parameters
    board_size = (5, 7)  # Size of the board in squares (X, Y)
    square_length = 0.032  # Length of each square edge in meters
    marker_length = 0.025  # Length of the marker edge in meters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((board_size[0], board_size[1]), square_length, marker_length, dictionary)

    if len(allCorners) > 0:
        # Perform the calibration
        cameraMatrixInit = np.array([[1.0, 0, 0.5 * imsize[1]],
                                    [0, 1.0, 0.5 * imsize[0]],
                                    [0, 0, 1.0]], dtype=np.float32)
        distCoeffsInit = np.zeros((5, 1), np.float32)

        try:
            _, cameraMatrix, distCoeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
                allCorners,
                allIds,
                board,
                imsize,
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

        # Save the calibration matrix and distortion coefficients to files
        matrix_file_path = os.path.join(save_dir, "camera_matrix_1.txt")
        coeffs_file_path = os.path.join(save_dir, "distortion_coefficients_1.txt")
        print()
        try:
            np.savetxt(matrix_file_path, cameraMatrix)
            np.savetxt(coeffs_file_path, distCoeffs)
        except Exception as e:
            print("Error occurred while saving the files:", e)

def capture_images(save_dir, calibration_count):
    # Create a context object
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device('141322250166')  # Replace 'DEVICE_SERIAL_NUMBER_1' with the actual serial number of the D435 camera
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    pipeline.start(config)

    try:
        image_count = 0
        color_images = []
        while True:
            # Wait for the next frameset
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert the color frame to BGR for OpenCV
            color_image = np.asanyarray(color_frame.get_data())
            color_image_with_ids = color_image.copy()

            # Detect markers and draw their IDs on the frame
            gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
            if len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(color_image_with_ids, corners, ids)

            # Display the color image with markers and corners
            cv2.imshow("Capture", color_image_with_ids)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            elif ids is not None:
                if len(ids) > 0:
                    # Save the raw color image
                    image_filename = os.path.join(save_dir, f"image_{image_count}.jpg")
                    cv2.imwrite(image_filename, color_image)
                    color_images.append(image_filename)
                    image_count += 1
                    print(f"Image saved: {image_filename}")
                    print(image_count)
                else:
                    print("No marker detected in the current frame.")

            if image_count == calibration_count:
                # Perform calibration using the obtained images
                # Read the Charuco board images and obtain corner coordinates and marker IDs
                board = cv2.aruco.CharucoBoard((5, 7), 0.032, 0.025, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
                allCorners, allIds, imsize = read_chessboards(color_images, board)
                
                # Perform calibration using the obtained corner coordinates and marker IDs
                calibrate_depth_sensor(allCorners, allIds, imsize,save_dir)

                # Exit the loop after calibration
                break

        # Stop the pipeline and release resources
        pipeline.stop()
        cv2.destroyAllWindows()

    except Exception as e:
        print("Error occurred during image capture:", e)

def calibrate_depth_sensor_folder(images_folder):
    # Define the Charuco board parameters
    board_size = (5, 7)  # Size of the board in squares (X, Y)
    square_length = 0.032  # Length of each square edge in meters
    marker_length = 0.025  # Length of the marker edge in meters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((board_size[0], board_size[1]), square_length, marker_length, dictionary)

    # Read the images in the folder
    image_files = os.listdir(images_folder)
    color_images = []
    for image_file in image_files:
        if image_file.endswith(".jpg") or image_file.endswith(".png"):
            image_path = os.path.join(images_folder, image_file)
            color_images.append(image_path)

    if len(color_images) > 0:
        # Read the Charuco board images and obtain corner coordinates and marker IDs
        allCorners, allIds, imsize = read_chessboards(color_images, board)
        print(allCorners, allIds, imsize)
        if len(allCorners) > 0:
            # Perform the calibration
            cameraMatrixInit = np.array([[1.0, 0, 0.5 * imsize[1]],
                                         [0, 1.0, 0.5 * imsize[0]],
                                         [0, 0, 1.0]], dtype=np.float32)
            distCoeffsInit = np.zeros((5, 1), np.float32)

            try:
                _, cameraMatrix, distCoeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
                    allCorners,
                    allIds,
                    board,
                    imsize,
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
            matrix_file_path = os.path.join(script_dir, "camera_matrix_2.txt")
            coeffs_file_path = os.path.join(script_dir, "distortion_coefficients_2.txt")
            try:
                np.savetxt(matrix_file_path, cameraMatrix)
                np.savetxt(coeffs_file_path, distCoeffs)
            except Exception as e:
                print("Error occurred while saving the files:", e)
        else:
            print("No corners detected in the provided images.")
    else:
        print("No images found in the folder.")



# Set the save directory for captured images
script_dir = os.path.dirname(os.path.abspath(__file__))
current_time = datetime.now().strftime("%Y%m%d%H%M%S")
save_dir = os.path.join(script_dir, f"images_{current_time}")#images_20230628172321
# save_dir = os.path.join(script_dir, f"images_20230628172321")

# os.makedirs(save_dir, exist_ok=True)
# Specify the number of images for calibration
calibration_count = 100

# Capture images and perform calibration
capture_images(save_dir, calibration_count)

#read images and callibrate
# calibrate_depth_sensor_folder(save_dir)