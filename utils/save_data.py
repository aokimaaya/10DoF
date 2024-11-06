import csv
import datetime
import os
import random
import time

import cv2
import numpy as np


class RecordData:
    def __init__(self, step_size, data_size, DymControl) -> None:
        self.directoryId = (
            str(datetime.datetime.now()).replace(" ", "_").replace(":", "-")
        )
        self.arm = DymControl

        self.time_step = 0
        self.data_rows = np.zeros((step_size, data_size))
        self.status=0
        self.comment=0
        self.cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 420)

        # self.cap_side = cv2.VideoCapture(3, cv2.CAP_DSHOW)
        # self.cap_side.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap_side.set(cv2.CAP_PROP_FRAME_HEIGHT, 420)

    def create_datadir(self, seq_num):
        # Flag Train Test
        self.seq_num=seq_num
        train_test = random.random()
        if train_test > 0.2:
            if not os.path.exists("data/wiping/%s" % self.directoryId):
                os.makedirs("data/wiping/%s" % self.directoryId)
            if not os.path.exists("data/wiping/%s/train" % self.directoryId):
                os.makedirs("data/wiping/%s/train" % self.directoryId)
            if not os.path.exists("data/wiping/%s/train/csv" % self.directoryId):
                os.makedirs("data/wiping/%s/train/csv" % self.directoryId)
            if not os.path.exists("data/wiping/%s/train/images" % (self.directoryId)):
                os.makedirs("data/wiping/%s/train/images" % (self.directoryId))
            if not os.path.exists(
                "data/wiping/%s/train/images/%s" % (self.directoryId, seq_num)
            ):
                os.makedirs(
                    "data/wiping/%s/train/images/%s" % (self.directoryId, seq_num)
                )
            if not os.path.exists(
                "data/wiping/%s/train/images_side" % (self.directoryId)
            ):
                os.makedirs("data/wiping/%s/train/images_side" % (self.directoryId))
            if not os.path.exists(
                "data/wiping/%s/train/images_side/%s" % (self.directoryId, seq_num)
            ):
                os.makedirs(
                    "data/wiping/%s/train/images_side/%s" % (self.directoryId, seq_num)
                )

            csv_path = "data/wiping/%s/train/csv/%s.csv" % (
                self.directoryId,
                seq_num,
            )
            image_path = "data/wiping/%s/train/images/%s/%s.png" % (
                self.directoryId,
                seq_num,
                "%s",
            )
            image_path_side = "data/wiping/%s/train/images_side/%s/%s.png" % (
                self.directoryId,
                seq_num,
                "%s",
            )
            excel_path="data/wiping/%s/%s.xlsx" % (
                self.directoryId,
                self.directoryId,
            )
        else:
            if not os.path.exists("data/wiping/%s" % self.directoryId):
                os.makedirs("data/wiping/%s" % self.directoryId)
            if not os.path.exists("data/wiping/%s/test" % self.directoryId):
                os.makedirs("data/wiping/%s/test" % self.directoryId)
            if not os.path.exists("data/wiping/%s/test/csv" % self.directoryId):
                os.makedirs("data/wiping/%s/test/csv" % self.directoryId)
            if not os.path.exists("data/wiping/%s/test/images" % (self.directoryId)):
                os.makedirs("data/wiping/%s/test/images" % (self.directoryId))
            if not os.path.exists(
                "data/wiping/%s/test/images/%s" % (self.directoryId, seq_num)
            ):
                os.makedirs(
                    "data/wiping/%s/test/images/%s" % (self.directoryId, seq_num)
                )
            if not os.path.exists(
                "data/wiping/%s/test/images_side" % (self.directoryId)
            ):
                os.makedirs("data/wiping/%s/test/images_side" % (self.directoryId))
            if not os.path.exists(
                "data/wiping/%s/test/images_side/%s" % (self.directoryId, seq_num)
            ):
                os.makedirs(
                    "data/wiping/%s/test/images_side/%s" % (self.directoryId, seq_num)
                )

            csv_path = "data/wiping/%s/test/csv/%s.csv" % (
                self.directoryId,
                seq_num,
            )
            image_path = "data/wiping/%s/test/images/%s/%s.png" % (
                self.directoryId,
                seq_num,
                "%s",
            )
            image_path_side = "data/wiping/%s/test/images_side/%s/%s.png" % (
                self.directoryId,
                seq_num,
                "%s",
            )

            excel_path="data/wiping/%s/%s.xlsx" % (
                self.directoryId,
                self.directoryId,
            )

        return csv_path, image_path, image_path_side,excel_path

    def record(self, image_path, image_path_side, servo):
        # Record Data for CSV
        print("time_step", self.time_step)
        ###self.arm.get_time(time.time(), self.time_step, "record_data")

        position = self.arm.signed(self.arm.getj(servo),servo)
        ###self.arm.get_time(time.time(), self.time_step, "record_getj")

        current = self.arm.getCurrent(servo)
        ###self.arm.get_time(time.time(), self.time_step, "record_getCurrent")

        self.data_rows[self.time_step][0] = self.time_step
        self.data_rows[self.time_step][1] = time.time()
        self.data_rows[self.time_step][2] = position[0]
        self.data_rows[self.time_step][3] = position[1]
        self.data_rows[self.time_step][4] = position[2]
        self.data_rows[self.time_step][5] = current[0]
        self.data_rows[self.time_step][6] = current[1]
        self.data_rows[self.time_step][7] = current[2]
        # print(self.data_rows[self.time_step])
        ###self.arm.get_time(time.time(), self.time_step, "record_datarows")

        # Record Image
        _, frame = self.cap.read()
        # cv2.imshow('Capture', frame)
        cv2.imwrite(image_path % (str(self.time_step).zfill(3)), frame)
        ###self.arm.get_time(time.time(), self.time_step, "record_image")

        # Record Image2
        # __, ###frame_side = self.cap_side.read()
        # cv2.imshow('Capture', frame)
        # cv2.imwrite(image_path_side % (str(self.time_step).zfill(3)), ###frame_side)
        ###self.arm.get_time(time.time(), self.time_step, "record_image_side")

        # Overwrite until the angle changes for the first time
        if self.arm.stats:

            self.arm.stats = 1
            self.time_step += 1
        # | (
        #     (
        #         abs(self.arm.wipe_angle[0] - self.arm.toSigned32(position[0], servo))
        #         > 200
        #     )
        #     & (
        #         abs(self.arm.wipe_angle[1] - self.arm.toSigned32(position[1], servo))
        #         > 200
        #     )
        #     & (
        #         abs(self.arm.wipe_angle[2] - self.arm.toSigned32(position[2], servo))
        #         > 15
        #     )
        # )

        else:
            print(
                self.arm.goal[servo],
                self.arm.toSigned32(position[0], servo),
                self.arm.toSigned32(position[1], servo),
                self.arm.toSigned32(position[2], servo),
            )
        return position, frame,##frame_side

    def save_data(self, csvfile_name, comments=0, trail=1):
        try:
            if comments:
                while 1:
                    self.status = input(
                        """Input desired status
                        Ok : 1
                    Not Ok : 0
                        Input : """
                    )
                    if self.status != "":
                        break
                while 1:
                    self.comment = input(
                        """Input additional comment
                        Input : """
                    )
                    if self.comment != "":
                        break
            else:
                self.status = 1
                self.comment = trail
            self.data_rows = self.data_rows[0 : self.time_step, :]
            f = open(csvfile_name, "w")
            f_csv = csv.writer(f)
            header = ["# " + __file__ + " " + str(datetime.datetime.now()), self.status, self.comment]
            f_csv.writerow(header)
            header = ["step", "time", "A1", "A2", "A3", "C1", "C2", "C3"]
            f_csv.writerow(header)
            f_csv.writerows(self.data_rows)
            self.time_step = 0
            comment = ""
            print("saved")
        except:
            print("error saving")
