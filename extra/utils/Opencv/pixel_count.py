import cv2
import numpy as np

class ArUcoDetector:
    def __init__(self, target_size=(400, 300), num_frames=5):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.target_size = target_size
        self.num_frames = num_frames
        self.frame_buffer = []

    def detect_markers(self, frame):
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)

        black_pixels = 0
        roi = None

        if markerIds is not None and len(markerIds) >= 4:
            roi_corners = np.vstack(markerCorners[:4])[:, 0, :]
            roi_corners = np.array(roi_corners, dtype=np.float32)
            dst_corners = np.array([[0, 0], [self.target_size[0], 0], [self.target_size[0], self.target_size[1]], [0, self.target_size[1]]], dtype=np.float32)
            M = cv2.getPerspectiveTransform(roi_corners, dst_corners)
            roi = cv2.warpPerspective(frame, M, self.target_size)

            # Add current frame to the buffer
            self.frame_buffer.append(roi)

            # Maintain buffer size
            if len(self.frame_buffer) > self.num_frames:
                self.frame_buffer.pop(0)

            # Compute average frame
            averaged_frame = np.mean(self.frame_buffer, axis=0).astype(np.uint8)

            gray_image = cv2.cvtColor(averaged_frame, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
            binary_image[np.all(averaged_frame == [0, 0, 255], axis=2)] = 255
            black_pixels = np.count_nonzero(binary_image == 0)
            inverted_frame = cv2.bitwise_not(binary_image)
            cv2.imshow("Inverted Frame", inverted_frame)
            cv2.imshow("ROI", averaged_frame)
            cv2.putText(
                frame,
                f"Black Pixels: {black_pixels}",
                (470, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
        frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        return frame, black_pixels, roi

# Usage example:
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
aruco_detector = ArUcoDetector()

while True:
    ret, frame = cap.read()
    frame, black_pixels, roi = aruco_detector.detect_markers(frame)

    cv2.imshow("ArUco Markers", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
