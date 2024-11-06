import cv2
import numpy as np

# Define the ArUco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Define the ArUco parameters
parameters = cv2.aruco.DetectorParameters()

# Define the target size of the extracted ROI
target_size = (400, 300)  # Adjust as needed

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
        dst_corners = np.array([[0, 0], [target_size[0], 0], [target_size[0], target_size[1]], [0, target_size[1]]], dtype=np.float32)

        # Calculate the perspective transformation matrix
        M = cv2.getPerspectiveTransform(roi_corners, dst_corners)

        # Apply the perspective transformation to extract the ROI
        roi = cv2.warpPerspective(frame, M, target_size)

        # Convert the cropped image to grayscale
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Convert the grayscale image to binary (black and white)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # Exclude the red pixels from the binary image
        binary_image[np.all(roi == [0, 0, 255], axis=2)] = 255

        # Count the number of black pixels within the ROI
        black_pixels = np.count_nonzero(binary_image == 0)

        # Invert the binary image
        inverted_frame = cv2.bitwise_not(binary_image)

        # Display the inverted frame
        cv2.imshow("Inverted Frame", inverted_frame)

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

    # Draw bounding boxes on detected markers
    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    return frame

# Create a video capture object
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    # Detect ArUco markers and display bounding boxes
    frame = detect_markers(frame)

    # Display the frame
    cv2.imshow("ArUco Markers", frame)

    # Check for 'q' key press to exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()
