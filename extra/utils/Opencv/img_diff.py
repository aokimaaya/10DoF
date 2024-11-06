import cv2
import numpy as np
import screeninfo

# Define the ArUco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Define the ArUco parameters
parameters = cv2.aruco.DetectorParameters()

# Function to resize the image
def resize_image(image, new_width):
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the scaling factor
    scale = new_width / width

    # Calculate the new height while maintaining the aspect ratio
    new_height = int(height * scale)

    # Resize the image using the calculated dimensions
    resized_image = cv2.resize(image, (new_width, new_height))

    return resized_image

# Calculate the target size based on screen size
screen = screeninfo.get_monitors()[0]  # Assuming a single monitor setup
target_size = int(screen.width * 0.25)

# Function to detect ArUco markers and extract ROI
def detect_markers(frame):
    # Detect the ArUco markers
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    # Check if any markers are detected
    if markerIds is not None and len(markerIds) >= 4:
        # Combine the first corners of all four markers
        roi_corners = np.vstack(markerCorners[:4])[:, 0, :]
        roi_corners = np.array(roi_corners, dtype=np.float32)

        # Define the destination corners for the warped perspective
        dst_corners = np.array([[0, 0], [target_size, 0], [target_size, target_size], [0, target_size]], dtype=np.float32)

        # Calculate the perspective transformation matrix
        M = cv2.getPerspectiveTransform(roi_corners, dst_corners)

        # Apply the perspective transformation to extract the ROI
        roi = cv2.warpPerspective(frame, M, (target_size, target_size))

        # Convert the cropped image to grayscale
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Convert the grayscale image to binary (black and white)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # Exclude the red pixels from the binary image
        binary_image[np.all(roi == [0, 0, 255], axis=2)] = 255

        # Count the number of black pixels within the ROI
        black_pixels = np.count_nonzero(binary_image == 0)
        inverted_frame = cv2.bitwise_not(binary_image)
        cv2.imshow("inverted_frame", inverted_frame)
        # Display the ROI and pixel count
        cv2.imshow("ROI", roi)
        cv2.putText(
            frame,
            f"Black Pixels: {black_pixels}",
            (470, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )

        return black_pixels, inverted_frame

    # Draw bounding boxes on detected markers
    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    return None, frame

def get_pixel_difference(image_before, image_after):
    # Resize the before image
    image_before = resize_image(image_before, target_size)
    image_after = resize_image(image_after, target_size)
    # cv2.imshow("image_before",image_before)
    # cv2.imshow("image_before",image_after)
    
    # Detect ArUco markers and get pixel count for the before image
    black_pixels_before, _ = detect_markers(image_before)

    # Resize the after image
    

    # Detect ArUco markers and get pixel count for the after image
    black_pixels_after, _ = detect_markers(image_after)

    # Calculate the difference in black pixelsz
    pixel_difference = black_pixels_after - black_pixels_before

    return pixel_difference

# Load the before and after images
image_before = cv2.imread(r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\before_image.jpg")
image_after = cv2.imread(r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\after_image.jpg")

# cv2.imshow("image_before",image_before)
# cv2.imshow("image_after",image_after)

# Calculate the pixel difference
difference = get_pixel_difference(image_before, image_after)
# difference = "0"
if difference is not None:
    print("Pixel Difference:", difference)
else:
    print("Failed to calculate pixel difference.")

# cv2.waitKey(0)
# cv2.destroyAllWindows()
