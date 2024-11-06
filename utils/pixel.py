import cv2
import numpy as np
import os
from datetime import datetime
import openpyxl

class ArUcoDetector:
    def __init__(self):
        self.capture_state = None
        self.before_frames = []
        self.after_frames = []
        self.data = []
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.target_size = (400, 300)  # Adjust as needed

    def detect_markers(self, frame):
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        # cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        black_pixels = 0
        roi = None
        inverted_frame=None
        sorted_corners = []  # Initialize with an empty list
        sorted_ids = []  # Initialize with an empty list
        sorted_indices = []  # Initialize with an empty list
        if markerIds is not None and len(markerIds) >= 4:
            sorted_indices = np.argsort(markerIds.flatten())
            sorted_corners = np.array(markerCorners)[sorted_indices]
            sorted_ids = np.array(markerIds.flatten(), dtype=np.int32)[sorted_indices]

            roi_corners = np.vstack(sorted_corners[:4])[:, 0, :]
            roi_corners = np.array(roi_corners, dtype=np.float32)
            dst_corners = np.array([[0, 0], [self.target_size[0], 0], [self.target_size[0], self.target_size[1]], [0, self.target_size[1]]], dtype=np.float32)
            # M = cv2.getPerspectiveTransform(roi_corners, dst_corners)
            # roi = cv2.warpPerspective(frame, M, self.target_size)
            # Define a bounding box around the ROI
            x, y, w, h = cv2.boundingRect(np.int32(roi_corners))
            
            # Crop the ROI from the frame
            roi = frame[y:y+h, x:x+w].copy()

            gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
            binary_image[np.all(roi == [0, 0, 255], axis=2)] = 255
            black_pixels = np.count_nonzero(binary_image == 0)
            inverted_frame = cv2.bitwise_not(binary_image)
            cv2.imshow("Inverted Frame", inverted_frame)
            cv2.imshow("ROI", roi)
            cv2.putText(
                frame,
                f"Pixels: {black_pixels}",
                (470, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

            cv2.polylines(frame, [np.int32(roi_corners)], isClosed=True, color=(0, 255, 0), thickness=2)
        # else:
        #     cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)


        if markerIds is not None and len(markerIds) > 0 and len(markerIds) < 4:
            for corners in markerCorners:
                cv2.polylines(frame, [np.int32(corners)], isClosed=True, color=(0, 0, 255), thickness=2)
            cv2.putText(
                frame,
                "Marker missing!",
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        frame = cv2.aruco.drawDetectedMarkers(frame, np.array(sorted_corners), np.array(sorted_ids))
        return frame, black_pixels, roi,inverted_frame, sorted_ids


    def capture_frames(self, save_folder,recordData, capture_state, angle=0):
        angle=str(angle)
        if capture_state == 'start':
            self.capture_state = 'before'
            self.before_frames = []
        elif capture_state == 'end':
            self.capture_state = 'after'
            self.after_frames = []
        if recordData != "norecord":
            cap = recordData.cap#
            t_step=recordData.seq_num
        else:
            cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)#recordData.cap#
            t_step="t_step"
        # cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn the autofocus off
        black_pixels_before = None
        black_pixels_after = None
        roi_before = None
        roi_after = None
        inverted_frame_a=None
        inverted_frame_b=None
        while True:
            ret, frame = cap.read()

            if ret:
                frame, black_pixels, roi,inverted_frame, markerIds = self.detect_markers(frame)

                if self.capture_state == 'before' and markerIds is not None and len(markerIds) >= 4 and black_pixels is not None:
                    self.before_frames.append(frame)
                    black_pixels_before = black_pixels
                    roi_before = roi
                    inverted_frame_b=inverted_frame
                    # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    # before_filename = f"before_{timestamp}_angle{angle}.jpg"
                    # roi_before_filename = f"before_ROI_{timestamp}_angle{angle}.jpg"
                    # cv2.imwrite(os.path.join(save_folder, before_filename), self.before_frames[-1])
                    # cv2.imwrite(os.path.join(save_folder, roi_before_filename), roi_before)
                    # self.data.append([timestamp, before_filename, roi_before_filename, angle, black_pixels_before])
                elif self.capture_state == 'after'and markerIds is not None and  len(markerIds) >= 4 and black_pixels is not None:
                    self.after_frames.append(frame)
                    black_pixels_after = black_pixels
                    roi_after = roi
                    inverted_frame_a=inverted_frame
                    # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    # after_filename = f"after_{timestamp}_angle{angle}.jpg"
                    # roi_after_filename = f"after_ROI_{timestamp}_angle{angle}.jpg"
                    # cv2.imwrite(os.path.join(save_folder, after_filename), self.after_frames[-1])
                    # cv2.imwrite(os.path.join(save_folder, roi_after_filename), roi_after)
                    # difference = abs(black_pixels_after - self.data[0][4])
                    # self.data[-1].extend([after_filename, roi_after_filename, angle, black_pixels_after, difference])

                cv2.imshow("Frame", frame)

            if cv2.waitKey(20) & 0xFF == ord('q'):
                print (self.data)
                break

        if self.capture_state == 'before' and roi_before is not None:
            timestamp = str(t_step)#+"_"+str(datetime.now().strftime("%Y%m%d_%H%M%S"))
            before_filename = f"before_{timestamp}_angle{angle}.png"
            roi_before_filename = f"before_ROI_{timestamp}_angle{angle}.png"
            # cv2.imwrite(save_folder % (before_filename), ###frame_side)

            cv2.imwrite(save_folder % (before_filename), self.before_frames[-1])
            cv2.imwrite(save_folder % (roi_before_filename), roi_before)
            cv2.imwrite(save_folder % ("bw_"+roi_before_filename),inverted_frame_b)
            if recordData != "norecord":
                self.data.append([timestamp, before_filename, roi_before_filename, angle, black_pixels_before])

            else:
                self.data.append([timestamp, before_filename, roi_before_filename, angle, black_pixels_before])
        elif self.capture_state == 'after' and roi_after is not None:
            timestamp = str(t_step)#.zfill(3)#+"_"+str(datetime.now().strftime("%Y%m%d_%H%M%S"))
            
            after_filename = f"after_{timestamp}_angle{angle}.png"
            roi_after_filename = f"after_ROI_{timestamp}_angle{angle}.png"
            cv2.imwrite(save_folder % (after_filename), self.after_frames[-1])
            cv2.imwrite(save_folder % (roi_after_filename), roi_after)
            cv2.imwrite(save_folder % ("bw_"+roi_after_filename),inverted_frame_a)
            difference = abs(black_pixels_after - self.data[0][4])
            if recordData != "norecord":
                self.data[-1].extend([after_filename, roi_after_filename, angle, black_pixels_after, difference])
            else:
                self.data[-1].extend([after_filename, roi_after_filename, angle, black_pixels_after, difference])

        # cap.release()
        cv2.destroyAllWindows()

    def save_data_to_excel(self,recordData, excel_file):
        if os.path.exists(excel_file):
            wb = openpyxl.load_workbook(excel_file)
            sheet = wb.active
        else:
            wb = openpyxl.Workbook()
            sheet = wb.active
            headers = ['Timestamp', 'Before Filename', 'Before ROI Filename', 'Angle', 'Before Black Pixels', 'After Filename', 'After ROI Filename', 'Angle', 'After Black Pixels', 'Difference',"ok","comment"]
            sheet.append(headers)
        if recordData != "norecord":
            self.data[-1].extend([recordData.status,recordData.comment])
        for row in self.data:
            # Convert the pixel array to a string representation
            # row[4] = str(row[4])
            # row[7] = str(row[7])
            # row[8] = str(row[8])
            sheet.append(row)

        wb.save(excel_file)
        self.data=[]


# Usage example:
# save_folder = "C:/Users/81809/Desktop/Three_DoF_Robotarm-main/Three_DoF_Robotarm/utils/fig/frames/%s/%s" % ("__50","%s")

# os.makedirs(save_folder, exist_ok=True)
# excel_file = save_folder + r"\frame_data.xlsx"

# aruco_detector = ArUcoDetector()
# aruco_detector.capture_frames(save_folder,"norecord", 'start', angle="50")  # Capture "before" frames with angle 45
# aruco_detector.capture_frames(save_folder,"norecord", 'end', angle="50")  # Capture "after" frames with angle 45
# aruco_detector.save_data_to_excel(excel_file)  # Save data to Excel file
