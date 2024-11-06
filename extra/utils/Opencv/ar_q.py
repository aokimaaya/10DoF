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
        roi_corners = np.reshape(roi_corners, (1, 4, 2))

        # Create a mask for the ROI
        roi_mask = np.zeros_like(frame)
        cv2.fillPoly(roi_mask, roi_corners.astype(int), (255, 255, 255))

        # Apply the mask to the frame
        roi = cv2.bitwise_and(frame, roi_mask)

        # Resize the ROI to the target size
        roi = cv2.resize(roi, target_size)

        # Convert the cropped image to grayscale
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Convert the grayscale image to binary (black and white)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # Exclude the red pixels from the black pixel count in the cropped image
        binary_image[np.all(roi == [0, 0, 255], axis=2)] = 255

        # Count the number of black pixels
        black_pixels = np.count_nonzero(binary_image == 0)

        # Display the frame, ROI, binary image, and black pixel count
        cv2.imshow("Frame", frame)
        cv2.imshow("ROI", roi)
        cv2.imshow("Binary Image", binary_image)
        cv2.putText(
            frame,
            f"Black Pixels: {black_pixels}",
            (470, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )

    # Draw bounding boxes on detected markers
    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

    return frame

# Create a video capture object
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    # Detect ArUco markers and display bounding boxes
    frame = detect_markers(frame)

    # Display the frame
    cv2.imshow("ArUco Markers", frame)

    # Check for 'q' key press to exit
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()
