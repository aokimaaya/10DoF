import json
import os

import numpy as np
import pandas as pd
import torch

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def normalize(data, in_range, out_range):
    in_range = np.array(in_range)
    denominator = in_range.max() - in_range.min()
    if denominator == 0:
        denominator = 1

    compressed = (data - in_range.min()) / denominator
    stretched = compressed * (out_range[1] - out_range[0]) + out_range[0]
    return stretched


def normalize_all(dataset, flag):
    normalized_csv = []

    if flag == "train":
        for i in range(len(dataset["train_minmax"])):
            normalized_angle = normalize(
                dataset["train_csv"][:, :, i], dataset["train_minmax"][i], [0, 1]
            )
            normalized_csv.append(normalized_angle)
            normalized_image = normalize(dataset["train_image"], [0, 255], [0, 1])
        print("Finished normalizing train data.")

    elif flag == "test":
        for i in range(len(dataset["train_minmax"])):
            normalized_angle = normalize(
                dataset["test_csv"][:, :, i], dataset["train_minmax"][i], [0, 1]
            )
            normalized_csv.append(normalized_angle)
            normalized_image = normalize(dataset["test_image"], [0, 255], [0, 1])
        print("Finished normalizing test data.")

    else:
        print("Value should be train or test.")

    normalized_csv = np.array(normalized_csv)
    normalized_csv = normalized_csv.transpose(1, 2, 0)

    return torch.Tensor(normalized_csv), torch.Tensor(normalized_image)


# list csv names
def find_csv_filenames(path_to_dir, suffix=".csv", test_train_range=0):
    # csv_en,csv_st="",""
    filenames = os.listdir(path_to_dir)
    csv_list = sorted(
        [
            int(filename.replace(suffix, ""))
            for filename in filenames
            if filename.endswith(suffix)
        ]
    )
    print("length = ", len(csv_list))
    print(
        path_to_dir[:-14],
        " list => [(csv_list,index)] ",
        (list(zip(csv_list, (list(range(0, len(csv_list))))))),
    )
    if test_train_range:
        while True:
            csv_st = input(" start value = ")
            if csv_st != "":
                break
        while True:
            csv_en = input(" end value = ")
            if csv_en != "":
                break
    else:
        csv_st = 0
        csv_en = len(csv_list)

    return csv_list[int(csv_st) : int(csv_en)]


def make_dataset(data_path, img_path, img_size=64, limit_seq=0, test_train_range=0):
    # Train
    # img_path = "/images_side/"
    train_image_path = data_path + "/train" + img_path + "%s/%s.png"
    train_list = find_csv_filenames(
        data_path + "/train/csv/", test_train_range=test_train_range
    )

    train_csv_path = data_path + "/train/csv/%s.csv"
    # train_list = sorted(
    #     list(map(lambda x: int(x), os.listdir(data_path + "/train" + img_path + "")))
    # )
    test_image_path = data_path + "/test" + img_path + "%s/%s.png"
    test_csv_path = data_path + "/test/csv/%s.csv"
    test_list = find_csv_filenames(
        data_path + "/test/csv/", test_train_range=test_train_range
    )

    # tt_list=sorted(train_list+test_list)
    # print(len(tt_list))

    # print()
    # print(tt_list,"len(angles_list)",len(angles_list))
    # angle_trails={}
    # for val in tt_list:
    #     # print(val)
    #     print(int((val)/trails),val)
    #     angle_trails[val]=angles_list[int((val)/trails)]
    # # angle_trails

    # selected_angles=list(range(50,10,-10))
    # print(selected_angles)
    # train_list_new=[]
    # test_list_new=[]
    # for tts in ["test","train"]:
    #     for ang_s in find_csv_filenames(path_dir+"\\"+tts+"\\csv"):
    #         if angle_trails[ang_s] in selected_angles:
    #             if tts == "train":
    #                 train_list_new.append(ang_s)
    #             else:
    #                 test_list_new.append(ang_s)

    print("train_list", train_list)
    train_csv, train_image, train_minmax = _get_datalist(
        train_list, train_image_path, train_csv_path, img_size, limit_seq
    )

    # Test

    # test_list = list(
    #     map(lambda x: int(x), os.listdir(data_path + "/test" + img_path + ""))
    # )
    print("test_list", test_list)
    test_csv, test_image, test_minmax = _get_datalist(
        test_list, test_image_path, test_csv_path, img_size, limit_seq
    )

    dataset = {
        "train_csv": train_csv,
        "train_image": train_image,
        "train_minmax": train_minmax,
        "test_csv": test_csv,
        "test_image": test_image,
        "test_minmax": test_minmax,
        "train_list": train_list,
        "test_list": test_list,
    }
    print("Finished extracting data.")
    return dataset


def toSigned32(n, servo="pro"):
    # print(n)
    n = int(n) & 0xFFFFFFFF
    # if n >= 501923:
    #     print("n",n , (n | (-(n & 0x80000000))) )
    return n | (-(n & 0x80000000))


def _csv_signed_angle(csv_df, ang_list):
    for a in ang_list:
        csv_df[a] = csv_df[a].apply(
            lambda row: toSigned32(row)
        )  # DymControl.toSigned32(row)
        csv_df[a] = csv_df[a].apply(
            lambda row: toSigned32(row)
        )  # DymControl.toSigned32(row)


def _get_datalist(lst, image_path, csv_path, img_size, limit_seq=0):
    csv_list = []
    image_list = []
    minmax_list = []
    colums_ = ["step", "time", "A1", "A2", "A3", "C1", "C2", "C3"]

    val_range = [limit_seq, 8]
    for l in lst:
        csv_df = pd.read_csv(
            csv_path % str(int(l)), skiprows=3, index_col=None, dtype="float64"
        )
        if limit_seq == 0:
            val_range = csv_df.shape
        else:
            val_range = [limit_seq, csv_df.shape[1]]
        print("csv_df.shape", csv_df.shape, "val_range", val_range)
        csv_df.columns = colums_  # ["step","time"	,"A1",	"A2",	"A3","C1","C2","C3"]
        _csv_signed_angle(csv_df, colums_[2:5])
        csv_df = csv_df.to_numpy()
        oneseq_csv, oneseq_img = [], []

        while len(csv_df) > 1:
            if (
                np.linalg.norm(
                    csv_df[1][2 : val_range[1]] - csv_df[0][2 : val_range[1]]
                )
                < 0.0001
            ):
                csv_df = np.append([csv_df[0]], csv_df[2 : val_range[1]], axis=0)

            if (
                np.linalg.norm(
                    csv_df[1][2 : val_range[1]] - csv_df[0][2 : val_range[1]]
                )
                < 0.0001
            ):
                csv_df = np.append([csv_df[0]], csv_df[2 : val_range[1]], axis=0)

            else:
                oneseq_csv.append(csv_df[0][2 : val_range[1]].tolist())
                ####original
                # img = cv2.imread(
                #     image_path % (str(int(l)), str(int(csv_df[0][0])).zfill(3))
                # )
                # resize_img = cv2.resize(img, (img_size, img_size))#img#
                # oneseq_img.append(resize_img)
                #####
                # img = cv2.imread(
                #     image_path % (str(int(l)), str(int(csv_df[0][0])).zfill(3))
                # )
                # resize_img = cv2.resize(img, (img_size, img_size))#img#
                oneseq_img.append([0])

                csv_df = csv_df[1:]

        else:
            if tuple(csv_df[0][2 : val_range[1]]) != tuple(oneseq_csv[-1]):
                oneseq_csv.append(csv_df[0][2 : val_range[1]].tolist())
                #####original
                # img = cv2.imread(
                #     image_path % (str(int(l)), str(int(csv_df[0][0])).zfill(3))
                # )
                # resize_img = cv2.resize(img, (img_size, img_size))
                # oneseq_img.append(resize_img)
                #######
                # img = cv2.imread(
                #     image_path % (str(int(l)), str(int(csv_df[0][0])).zfill(3))
                # )
                # resize_img = cv2.resize(img, (img_size, img_size))
                oneseq_img.append([0])
                #######

        oneseq_csv = _synchronize_shape(oneseq_csv, val_range[0] + 1)
        oneseq_img = _synchronize_shape(oneseq_img, val_range[0] + 1)

        csv_list.append(oneseq_csv)
        image_list.append(oneseq_img)

    csv_list = np.array(csv_list, dtype=np.float64)

    csv_list = np.array(csv_list, dtype=np.float64)
    image_list = np.array(image_list)

    for i in range(val_range[1] - 2):
        min_angle = np.min(csv_list[:, :, i])
        max_angle = np.max(csv_list[:, :, i])
    for i in range(val_range[1] - 2):
        min_angle = np.min(csv_list[:, :, i])
        max_angle = np.max(csv_list[:, :, i])
        minmax_list.append([min_angle, max_angle])

    print("csv_list:{}, image_list:{}".format(csv_list.shape, image_list.shape))
    print("minmax_list:{}".format(minmax_list))
    return csv_list, image_list, minmax_list


def _synchronize_shape(dt, size):
    if len(dt) > size:
        dt = dt[:size]
    elif len(dt) < size:
        for i in range(size - len(dt)):
            dt.append(dt[-1])
    return dt
    if len(dt) > size:
        dt = dt[:size]
    elif len(dt) < size:
        for i in range(size - len(dt)):
            dt.append(dt[-1])
    return dt


def save_configuration(data_dir, epoch, batchsize, train_minmax, test_minmax):
    config = {}
    config["data_dir"] = data_dir
    config["epoch"] = epoch
    config["batchsize"] = batchsize
    config["train_minmax"] = train_minmax
    config["test_minmax"] = test_minmax
    config["data_dir"] = data_dir
    config["epoch"] = epoch
    config["batchsize"] = batchsize
    config["train_minmax"] = train_minmax
    config["test_minmax"] = test_minmax

    return config


def load_json(path):
    with open(path) as f:
        config = json.load(f)
    return config
