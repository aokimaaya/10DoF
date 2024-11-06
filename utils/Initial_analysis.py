import argparse
import datetime
import json
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy
import torch
from torch.utils.data import TensorDataset

from extra.models.LSTM_with_CAE import BASE
from extra.utils.get_losses import get_loss
from tool.dym_setup import Control
from utils.edit_data import (
    load_json,
    make_dataset,
    normalize,
    normalize_all,
    save_configuration,
)
from utils.generate_angles import generateAngles
from utils.save_data import RecordData  # as recordData

# Syntax:
# python -m utils.Initial_analysis data/wiping/2023-09-17_21-12-17.670129_LF_Data_ --diff 7 --trail 10
# 2023-01-19_23-54-29.152547_B-20-_20-50_50F --diff 7 --trail 2
# 2023-01-09_20-41-40.094885-B-AC-_50-20x5x10x7_256 --diff 7 --trail 10

# 2023-01-19_23-54-29.152547_B-20-_20-50_50F --diff 7 --trail 2
#
# 2023-01-20_00-34-36.824241_B-50-_50-20x5x2x7_20F_256 --diff 7 --trail 2


# Small_Flat_ --diff 150 --trail 1
#
# 2023-01-09_20-41-40.094885-B-AC-_50-20x5x10x7_256 --diff 7 --trail 10
#
# Small_Flat_ --diff 150 --trail 1
#
# Small_Curved_  --diff 11 --trail 5
#
# Small_Incline_ --diff 7 --trail 10
# ###################
# 2023-01-09_20-41-40.094885-B-AC-_50-20x5x10x7_256 --diff 7 --trail 10
# 2023-01-20_15-44-45.316369_DMB-50-_0F --diff 1 --trail 2
#
# 2023-01-19_23-54-29.152547_B-20-_20-50_50F --diff 7 --trail 2
#
# 2023-01-09_20-41-40.094885-B-AC-_50-20x5x10x7_256 --diff 7 --trail 10
#
##DM datasets
# 2023-01-19_23-54-29.152547_B-20-_20-50_50F --diff 7 --trail 2
#
# 2023-01-20_00-34-36.824241_B-50-_50-20x5x2x7_20F_256 --diff 7 --trail 2

# 2023-01-19_23-54-29.152547_B-20-_20-50_50fail --diff 7 --trail 2
# ##########

#
# 2023-01-20_01-36-02.327921_B35_50diffx5x2x7_256 --diff 7 --trail 2
#
# 2023-01-09_20-41-40.094885-B_50-20x5x10x7_256 --diff 4 --trail 10
# 2023-01-08_22-11-06.071465-S_newx5x5x7 --diff 7 --trail 5
# 2023-01-08_20-39-06.996972 --diff 7 --trail 1
#
# 2023-01-07_20-40-52.235052-B_45-2025!x5x5x6 --diff 6 --trail 5 --test_train_range 0
# 2023-01-04_23-16-08.814402-B_65-20x5x10x10 --diff 10 --trail 10
# 2023-01-03_22-36-40.848797-B_55-20x3x6x8  --diff 8 --trail 6
# 2022-12-21_01-29-44.417279-first-B --diff 5 --trail 1
# python -m utils.Initial_analysis data/wiping/2022-12-29_18-15-19.515219-B_65-30x5x5x8 --diff 9 --trail 10
# 2022-12-25_21-40-54.834611-B_inc_65-20x5x5x10 --diff 10 --trail 5
# 2022-12-21_19-43-58.693018-B_60-20_5x5 --diff 5 --trail 5
# 2022-12-07_13-53-10.861204-Inc_5x5x7a --diff 7 --trail 5
# 2022-11-24_01-04-21.389319-Wipec-auto
# straight_200_mb
# python -m utils.Initial_analysis data/wiping/2022-11-19_08-24-38.120429-auto
# python -m utils.Initial_analysis ~/Inflatable_Data/data/wiping/straight_200_mb
#
# _small_board


# remote server
# python -m utils.Initial_analysis ~/Inflatable_Data/data/wiping/B_45-2025qx5x5x6 --diff 6 --trail 5

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


# list csv names
import os

from utils.edit_data import _get_datalist


def find_csv_filenames(path_to_dir, suffix=".csv"):
    # path_dir=r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\data\wiping"

    filenames = os.listdir(path_to_dir)
    return [
        int(filename.replace(suffix, ""))
        for filename in filenames
        if filename.endswith(suffix)
    ]


def Analysis(data_dir, q, trail, test_train_range):

    dataset = make_dataset(data_dir, "/images/", 64, limit_seq=299)
    print(dataset["train_csv"].shape)
    print(dataset["train_csv"][0, :, 0])

    liveplot(dataset, data_dir, q, trail)


def liveplot(dataset, path_dir, q, trail):
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib import cm

    des_dir = path_dir[12:]
    if "B-" in des_dir:
        des_angle_ = des_dir[des_dir.index("B-") + 2 : des_dir.index("B-") + 4]
        if des_angle_ == "AC":
            name = "Big_Incline_"
        else:
            name = "Fixed Motion "
    else:
        des_angle_ = ""  # des_dir[des_dir.index("B-")+2:des_dir.index("B-")+4]
        name = des_dir

    fontsize = 60
    font = {"family": "normal", "weight": "bold", "size": fontsize}
    plt.rc("font", **font)
    train_list = find_csv_filenames(path_dir + r"\train\csv")
    print(len(train_list))

    test_list = find_csv_filenames(path_dir + r"\test\csv")
    print(len(test_list))
    start_ang = 50
    tick = -5
    angles_list = list(
        range(start_ang, start_ang + q * tick, tick)
    )  ##start_ang,start_ang+q*5,5))#q))# [60,55,50,45,40,35,0]#list(range(54))#
    print(angles_list)
    tt_list = sorted(train_list + test_list)
    print(len(tt_list))

    print()
    print(tt_list, "len(angles_list)", len(angles_list))
    angles_trails = {}


    for vali, val in enumerate(tt_list):
        # print(val)
        if q == 1:
            angles_trails[val] = val  # angles_list[int((val-tt_list[0])/trail)]
        else:
            print(int((val ) / trail), val, tt_list[0])# - tt_list[0]
            angles_trails[val] = angles_list[int((vali) / trail)]# - tt_list[0]
    print(angles_trails)

    for j in range(6):
        plt.figure(figsize=[35, 20])
        # for dts in ["train","test"]:
        # datas=dataset[dts+"_csv"]
        datas_train = dataset["train_csv"]
        print(datas_train.shape)
        datas_test = dataset["test_csv"]
        print(datas_test.shape)
        datas = np.concatenate(
            (datas_train, datas_test), axis=0
        )  # datas_train+datas_test np.add np.concatenate
        print(datas.shape)
        train_list = dataset["train_list"] + dataset["test_list"]
        print("tt_list(train_list*)", train_list)
        print("datas.shape", datas.shape)

        ang_list = ["A1", "A2", "A3", "C1", "C2", "C3"]
        units = ["Angle in degree"] * 3 + ["Current in milli-ampere"] * 3
        # if j%3==0:
        # [128, 80])
        # color = iter(cm.rainbow(np.linspace(0, 1, datas.shape[0])))
        print("int(len(train_list)/q)+1", int((max(train_list) + 1) / q) + 1)
        colr = cm.tab20(range(q + 1))  # int((max(train_list)+1)/q)+1)
        # colr=  cm.tab20(range(42))
        # for i in range(50):
        s = 0
        i = 0
        for i, data in enumerate(datas):
            print(data.shape, train_list[i], angles_trails[train_list[i]])
            # fmt = lambda x1: "{:.3f}".format(x1)
            # for j,dat in enumerate(data[:,:3]):
            # if i %q ==0:
            if q == 1:
                cc = i
            else:
                # cc=int((train_list[i]+1)/trail)
                cc = int((train_list[i] - tt_list[0]) / trail)
            # print(cc)

            print(colr[int(cc)], i, "cc", cc)
            c = colr[int(cc)]
            if j < 3:
                y = data[:, j] * 0.00035862
            else:
                y = data[:, j]
            if 1:#(train_list[i] - tt_list[0]) % trail == 0:
                plt.plot(
                    range(0, len(y)),
                    y,
                    lw=2,
                    c=c,
                    # label=str(angles_trails[train_list[i]])
                )
                plt.scatter(
                    range(0, len(y)),
                    y,
                    s=50,
                    marker="o",
                    color=c,
                    # label=str(angles_trails[train_list[i]]),
                )

                plt.annotate(
                angles_trails[train_list[i]],  # this is the text
                (
                    int(len(y)/2),
                    data[int(len(y)/2), j],
                ),  # these are the coordinates to position the label
                textcoords="offset points",
                fontsize=20,  # how to position the text
                xytext=(0, 0),  # distance from text to points (x,y)
                # rotation=90,
                # ha="center",
                )
            else:
                pass
            s = 0
            if j < 3:
                y_ = datas[:, :, j] * 0.00035862
            else:
                y_ = datas[:, :, j]
            plt.legend(
                loc="lower center",
                bbox_to_anchor=[0.5, 1],
                ncol=q,  # int(q/2)+1,
                fontsize=fontsize - 8,
                markerscale=5,
                title=name + "-" + des_angle_ + "-" + ang_list[j],  # dts +
            )
            # plt.scatter(
            #     range(range(0,len(data))),
            #     y_live_sosfilt,
            #     lw=3,
            #     label="LiveLFilter-" + name + "-" + i + "-" + str(j),
            # )
            print(
                "int((np.max(y_)-np.min(y_))*tick/(len(y_))",
                int((np.max(y_) - np.min(y_)) * tick / (len(y_))),
            )

            # plt.yticks(
            #     np.arange(
            #         np.min(
            #         y_
            #         ),
            #         np.max(
            #         y_
            #         )+10,
            #         10#1000,
            #     ),
            #     #rotation=60,
            #     size=fontsize-8,
            # )
            time_xx = np.array(range(0, data.shape[0]))  # *0.0625
            # plt.xticks(time_xx[::160], 50), rotation=60, fontsize=fontsize-8)
            labels = np.array(time_xx[::80]) * 0.0625
            # # plt.plot(x,y, 'r')
            plt.xticks(time_xx[::80], labels)
            plt.xlabel("Time in seconds")
            plt.ylabel(ang_list[j] + units[j])
            #

        plt.tight_layout()
        # plt.pause(0.25)
        plt.savefig(
            "./utils/fig/Thesis_Pictures/new_time/"
            + str(name)
            + des_angle_
            + ang_list[j]
            + "_.png"
        )
        print("Check the plots")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("data_dir", type=str, default="data/normalized_screwing/train")
    parser.add_argument("--diff", type=int, default=1)
    parser.add_argument("--trail", type=int, default=1)
    parser.add_argument("--test_train_range", type=int, default=0)

    args = parser.parse_args()
    print(args)
    Analysis(
        data_dir=args.data_dir,
        q=args.diff,
        trail=args.trail,
        test_train_range=args.test_train_range,
    )
