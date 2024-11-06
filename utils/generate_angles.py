import csv
import datetime
import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image

from utils.edit_data import normalize


class generateAngles:
    def __init__(self, step_size, data_size, DymControl, cam_num) -> None:
        self.directoryId = (
            str(datetime.datetime.now()).replace(" ", "_").replace(":", "-")
        )
        self.arm = DymControl
        
        # self.record_data = RecordData()
        # self.ang_len=data_size-2
        self.time_step = 0
        self.data_rows = np.zeros((step_size, data_size + 32))

        self.cap = cv2.VideoCapture(cam_num, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    def create_logdir(self):
        logs_path = "logs/prediction_logs/wiping/{}".format(self.directoryId)
        if not os.path.exists(
            "logs/prediction_logs/wiping/{}".format(self.directoryId)
        ):
            os.makedirs("logs/prediction_logs/wiping/{}".format(self.directoryId))
        if not os.path.exists(
            "logs/prediction_logs/wiping/{}/real".format(self.directoryId)
        ):
            os.makedirs("logs/prediction_logs/wiping/{}/real".format(self.directoryId))
        if not os.path.exists(
            "logs/prediction_logs/wiping/{}/pred".format(self.directoryId)
        ):
            os.makedirs("logs/prediction_logs/wiping/{}/pred".format(self.directoryId))
        return logs_path

    def record(self, servo, ang_len, ls=1, img_size=64):
        # Record Data for CSV
        position = self.arm.getj(servo)  # "pro"
        for i, ang in enumerate(position):
            position[i] = self.arm.toSigned32(ang)

        if ang_len > 3:
            current = self.arm.getCurrent(servo)
            # print("current",current)
            position = position[::-1] + current[::-1]
        # Record Image
        if ls:
            _, frame = self.cap.read()
            frame = cv2.resize(frame, (img_size, img_size))
            return position, frame
        else:

            return position

    def prediction(
        self, x_image, x_position, train_rnn_hidden, model, dataset, img_size
    ):
        # print("len(dataset['train_minmax'])",len(dataset["train_minmax"]))

        # Normalize
        x_image = np.array(normalize(x_image, [0, 255], [0, 1])).reshape(
            1, img_size, img_size, 3
        )
        x_image = np.transpose(x_image, (0, 3, 1, 2))
        x_image = torch.Tensor(x_image)

        for i in range(len(dataset["train_minmax"])):
            x_position[i] = normalize(x_position[i], dataset["train_minmax"][i], [0, 1])
        x_position = torch.Tensor(x_position).reshape(1, -1)

        # Prediction
        y_image, y_image_next, y_position, train_rnn_hidden = model(
            x_image, x_position, train_rnn_hidden
        )
        # Denormalize
        y_image = normalize(y_image, [0, 1], [0, 255])
        y_image_next = normalize(y_image_next, [0, 1], [0, 255])
        for i in range(len(dataset["train_minmax"])):
            y_position[0, i] = normalize(
                y_position[0, i], [0, 1], dataset["train_minmax"][i]
            )
        # print("Denormalize",y_position)

        # denormalize xvalues for plotting
        for i in range(len(dataset["train_minmax"])):
            # print("before",x_position[0, i])
            x_position[0, i] = normalize(
                x_position[0, i], [0, 1], dataset["train_minmax"][i]
            )
            # print("after",x_position[0, i])

        # Save Data
        self.data_rows[self.time_step][0] = self.time_step
        self.data_rows[self.time_step][1] = time.time()
        self.data_rows[self.time_step][2] = x_position[0][0]
        self.data_rows[self.time_step][3] = x_position[0][1]
        self.data_rows[self.time_step][4] = x_position[0][2]
        self.data_rows[self.time_step][5] = y_position[0][0]
        self.data_rows[self.time_step][6] = y_position[0][1]
        self.data_rows[self.time_step][7] = y_position[0][2]
        if len(dataset["train_minmax"]) > 3:
            self.data_rows[self.time_step][8] = x_position[0][3]
            self.data_rows[self.time_step][9] = x_position[0][4]
            self.data_rows[self.time_step][10] = x_position[0][5]
            self.data_rows[self.time_step][11] = y_position[0][3]
            self.data_rows[self.time_step][12] = y_position[0][4]
            self.data_rows[self.time_step][13] = y_position[0][5]

        # Reshape Data for Analysis
        y_image = y_image.detach().numpy()[0]
        y_image = np.transpose(y_image, (1, 2, 0))
        y_image_next = y_image_next.detach().numpy()[0]
        y_image_next = np.transpose(y_image_next, (1, 2, 0))

        y_position = y_position.detach().numpy()[0]
        self.time_step += 1
        return y_image, y_image_next, y_position, train_rnn_hidden

    def prediction_lstm(self, x_position, train_rnn_hidden, model, dataset, slope):
        # print("len(dataset['train_minmax'])",len(dataset["train_minmax"]))

        for i in range(len(dataset["train_minmax"])):
            x_position[i] = normalize(x_position[i], dataset["train_minmax"][i], [0, 1])
        x_position = torch.Tensor(x_position).reshape(1, -1)

        # Prediction
        y_position, train_rnn_hidden = model(x_position, train_rnn_hidden)
        # Denormalize
        for i in range(len(dataset["train_minmax"])):
            y_position[0, i] = normalize(
                y_position[0, i], [0, 1], dataset["train_minmax"][i]
            )
        # print("Denormalize",y_position)

        # denormalize xvalues for plotting
        for i in range(len(dataset["train_minmax"])):
            # print("before",x_position[0, i])
            x_position[0, i] = normalize(
                x_position[0, i], [0, 1], dataset["train_minmax"][i]
            )
            # print("after",x_position[0, i])

        # Save Data
        self.data_rows[self.time_step][0] = self.time_step
        self.data_rows[self.time_step][1] = time.time()
        self.data_rows[self.time_step][2] = x_position[0][0]
        self.data_rows[self.time_step][3] = x_position[0][1]
        self.data_rows[self.time_step][4] = x_position[0][2]
        self.data_rows[self.time_step][5] = y_position[0][0]
        self.data_rows[self.time_step][6] = y_position[0][1]
        self.data_rows[self.time_step][7] = y_position[0][2]
        if len(dataset["train_minmax"]) > 3:
            self.data_rows[self.time_step][8] = x_position[0][3]
            self.data_rows[self.time_step][9] = x_position[0][4]
            self.data_rows[self.time_step][10] = x_position[0][5]
            self.data_rows[self.time_step][11] = y_position[0][3]
            self.data_rows[self.time_step][12] = y_position[0][4]
            self.data_rows[self.time_step][13] = y_position[0][5]
            self.data_rows[self.time_step][14]=float(slope)
        # print(self.data_rows[self.time_step])
        # save image
        # _, frame = self.cap.read()
        # # frame = cv2.resize(frame, (320, 320))
        # self.save_image(frame, frame, logs_path)
        # cv2.imwrite(logs_path+"real/"+str(self.time_step).zfill(3) + ".png", frame)
        y_position = y_position.detach().numpy()[0]
        self.time_step += 1
        return y_position, train_rnn_hidden

    def save_image(self, real, pred, image_path):
        pil_real = Image.fromarray(real)
        # pil_pred = Image.fromarray(pred)

        pil_real.save(image_path + "/real/" + str(self.time_step).zfill(3) + ".png")
        # pil_pred.save(image_path + '/pred/' + (str(self.time_step).zfill(3)) + ".png")

    def show_image(self, x_image, y_new_image):
        img_realworld = cv2.resize(
            x_image[0].astype(np.uint8), (256, 256), interpolation=cv2.INTER_LANCZOS4
        )
        img_predict = cv2.resize(
            y_new_image[0].astype(np.uint8),
            (256, 256),
            interpolation=cv2.INTER_LANCZOS4,
        )
        mergeImg = np.hstack((img_realworld, img_predict))
        # cv2.imshow("Image", mergeImg)

    def draw_results(self, logs_path, log_dir, comment=0):

        self.save_data(logs_path, log_dir, comment)
        # Position
        xyz = ["A1", "A2", "A3"]
        plt.figure(figsize=(12, 16))
        steps = list(range(len(self.data_rows)))
        x_positions = np.array(self.data_rows[:, 2:5])
        y_positions = np.array(self.data_rows[:, 5:8])
        for i in range(3):
            one_position = np.concatenate(
                (
                    x_positions[:, i].reshape(-1, 1),
                    y_positions[:, i].reshape(-1, 1),
                ),
                axis=1,
            )
            plt.subplot(3, 1, i + 1)
            plt.plot(steps, one_position, linestyle="dashed")
            plt.title(xyz[i])

        plt.tight_layout()
        plt.savefig(logs_path + "/angles_.png")
        print("Saved in {}/angles_.png".format(logs_path))
        # draw current
        plt.figure(figsize=(12, 16))
        xyz = ["C1", "C2", "C3"]
        x_positions = np.array(self.data_rows[:, 8:11])
        c_positions = np.array(self.data_rows[:, 11:14])
        for i in range(3):
            current = np.concatenate(
                (
                    x_positions[:, i].reshape(-1, 1),
                    c_positions[:, i].reshape(-1, 1),
                ),
                axis=1,
            )
            plt.subplot(3, 1, i + 1)
            plt.plot(steps, current, linestyle="dashed")
            plt.title(xyz[i])

        plt.tight_layout()
        plt.savefig(logs_path + "/current_.png")
        print("Saved in {}/current_.png".format(logs_path))

        for i in ["angle", "current"]:
            # data_.plot(x="step",y=["train_"+i+"_loss","test_"+i+"_loss"])

            steps = list(range(len(self.data_rows)))
            if i == "angle":

                x_positions = np.array(self.data_rows[:, 2:5])
                y_positions = np.array(self.data_rows[:, 5:8])
            else:
                x_positions = np.array(self.data_rows[:, 8:11])
                y_positions = np.array(self.data_rows[:, 11:14])
            for j in range(3):
                plt.figure(figsize=[30, 18])
                # current = np.concatenate(
                #     (x_positions[:, j].reshape(-1, 1),
                #     y_positions[:, j].reshape(-1, 1),
                #     ),
                #     axis=1,
                # )
                # plt.subplot(3, 1, i + 1)
                # plt.plot(steps, current, linestyle="dashed")
                # plt.title(xyz[i])
                plt.plot(
                    steps,
                    x_positions[:, j].reshape(-1, 1),
                    "-",
                    label="Actual_" + i + "_" + str(j),
                )
                plt.plot(
                    steps,
                    y_positions[:, j].reshape(-1, 1),
                    "-",
                    label="predicted_" + i + "_" + str(j),
                )

                plt.legend(
                    loc="lower center",
                    bbox_to_anchor=[0.5, 1],
                    ncol=2,
                    fontsize=24,
                    title=log_dir + i,
                )

                # plt.xticks(np.arange(0,max(data_["step"])+200,200),rotation = 60,size=16)
                # plt.yticks(
                #     np.arange(
                #         0,
                #         max(
                #             max(data_["train_" + i + "_loss"]),
                #             max(data_["test_" + i + "_loss"]),
                #         ),
                #         0.0001,
                #     ),
                #     size=20,
                # )

                plt.xlabel("Timesteps")
                plt.ylabel(i)
                # plt.yscale("log")
                # plt.xscale("log")
                #

                plt.tight_layout()
                # plt.pause(0.25)

                plt.savefig(
                    logs_path
                    + "/predict_"
                    + str(log_dir)
                    + "-"
                    + i
                    + "_"
                    + str(j)
                    + "_.pdf",
                    dpi=300,
                    format="pdf",
                )

    def save_data(self, csvfile_name, log_dir, comments):
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
            self.comment = 1
        # self.data_rows = self.data_rows[0:self.time_step, :]
        f = open(csvfile_name + "/predict_" + log_dir + ".csv", "w")
        f_csv = csv.writer(f)
        header = ["# " + __file__ + " " + str(datetime.datetime.now()), self.status, self.comment]
        f_csv.writerow(header)
        header = [
            "step",
            "time",
            "A1",
            "A2",
            "A3",
            "A1p",
            "A2p",
            "A3p",
            "C1",
            "C2",
            "C3",
            "C1p",
            "C2p",
            "C3p",
            "Slope"
        ] + ["Hid" + str(k) for k in range(25)]
        f_csv.writerow(header)
        f_csv.writerows(self.data_rows)
        self.time_step = 0
        comment = ""
        print("saved")
