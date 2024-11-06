import argparse
import os
import time

import torch

from models.LSTM_6_working_incline import BASE
from tool.dym_setup import Control
from utils.edit_data import load_json, normalize
from utils.generate_angles import generateAngles
from utils.save_data import RecordData  # as recordData

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


# Syntax:
# python -m src.motion_generation_lstm_predict2 logs\learning_logs\wiping\20221121_030630-LSTMauto256
# 20221121_030630-LSTMauto256 (working for incline)
#
# 20221108_200939
#
# python -m src.motion_generation logs\learning_logs\wiping\20221118_211323-xtra --cam_num 1
# 20221106_194323


def generate_motion(log_dir, step, input_param, model_path, cam_num):
    try:
        # Lord Setting
        dataset = load_json(os.path.join(log_dir, "dataset.json"))
        ang_len = len(dataset["train_minmax"])
        DymControl = Control()
        getAngles = generateAngles(
            step, 8, DymControl, cam_num
        )  # assign proper can number side view = 3 , top view = 1, inbuilt cam = 0
        logs_path = getAngles.create_logdir()
        print("Finished Loading settings")

        # Initialize Position
        nangle = 2048
        servo = "pro"
        DymControl.Enable(servo)
        # goal = dict(xm=[nangle, nangle, nangle], pro=DymControl.getj(servo))
        # change for every motion generation model
        low_pos = [213849, 43113, -6142]
        # high_pos=[149437,39938,2587]
        hover_pos = [204995, 13147, 2566]
        # low _pos1=[203536,22670,2586]
        # high_pos1=[149437,39938,2587]

        Wipe_inc_trj = [hover_pos, low_pos]

        DymControl.Initialize(Wipe_inc_trj)
        goal = DymControl.goal

        # initialize low pass filter
        d = 2
        fs = 20
        Wn = 2
        DymControl.LiveLPF_initialize(d=d, fs=fs, Wn=Wn)

        # Prediction Setting
        model = BASE(ang_len)
        print("model_log:", model_path)
        model_path = log_dir + "/models/" + model_path + ".pth"
        model.load_state_dict(torch.load(model_path))
        model.to("cpu")
        model.eval()
        print("Initialize.")
        print("Start online prediction at input_param={}".format(input_param))
        if getch() == chr(0x1B):
            for servo in DymControl.servos:
                DymControl.Disable(servo)
            quit()

        # Start Online Prediction
        q = 2  # unit step
        train_rnn_hidden = None
        for t in range(step):
            if t % q == 0:

                position = getAngles.record(servo, ang_len, 0)
            else:
                x_position, train_rnn_hidden = y_position, train_rnn_hidden

            if t == 0:
                x_position = position
                # x_image = image

            else:
                x_position = input_param * position + (1 - input_param) * y_position

            # Prediction

            y_position, train_rnn_hidden = getAngles.prediction_lstm(
                x_position, train_rnn_hidden, model, dataset
            )

            # Visualize
            # getAngles.show_image(x_image, y_image_next)
            if t % q == 0:
                yposition, ycurrent = y_position[:ang_len], y_position[ang_len:]
                # Move Dym
                if servo == "xm":
                    xm_align_angles = DymControl.rounderror(yposition, 245.0, servo)

                    print("y_position", xm_align_angles, yposition, ycurrent)
                    print(
                        "XM ",
                        DymControl.value2angle(xm_align_angles, "xm"),
                        "PRO",
                        DymControl.value2angle(yposition, "pro"),
                    )

                    # y_position[2]=0
                    # goal[servo]=y_position
                    # xm_align_angles[2]=2048
                    goal[servo] = xm_align_angles
                if servo == "pro":
                    # xm_align_angles=DymControl.rounderror(y_position,245.0,servo)
                    # pro_filtered_values=DymControl.Filter(yposition[:3],"pro")
                    if t == 0:
                        pro_current_values = DymControl.getj("pro")
                        initial_pro_angles = DymControl.value2angle(
                            pro_current_values, "pro"
                        )
                        print("present_pro_angles", initial_pro_angles)
                        DymControl.sin_movej(
                            pro_current_values, yposition[:3], 0.03, servo
                        )
                        xposition = []
                    for i in range(len(x_position)):
                        xposition.append(
                            normalize(x_position[i], [0, 1], dataset["train_minmax"][i])
                        )

                    # print("y_position",yposition,pro_filtered_values)#,xm_align_angles)
                    print(
                        "angle actual",
                        DymControl.value2angle(xposition[:3], "pro"),
                        "angle predicted",
                        DymControl.value2angle(yposition[:3], "pro"),
                        "current actual",
                        xposition[3:],
                        "current predicted",
                        yposition[3:],
                    )  # "XM ",DymControl.value2angle(pro_filtered_values,"pro")

                    # y_position[2]=0
                    pro_filtered_values = yposition[:3]  #
                    goal[servo] = pro_filtered_values  # yposition[:3]#

                # DymControl.movej(goal, servo)
                # time.sleep(0.05)

        # Draw Results
        # DymControl.Disable(servo)
        getAngles.draw_results(logs_path, log_dir.split("\\")[-1])

    except KeyboardInterrupt:
        print("Interrupted")
        for servo in DymControl.servos:
            DymControl.Disable(servo)
    finally:
        print("Finish Prediction.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", type=str, help="Path to logging directory.")
    parser.add_argument("--step", type=int, default=256)
    parser.add_argument("--input_param", default=1)
    parser.add_argument("--model_path", type=str, default="model_final")
    parser.add_argument("--cam_num", type=int, default=3)
    args = parser.parse_args()
    print(args)
    generate_motion(
        log_dir=args.log_dir,
        step=args.step,
        input_param=args.input_param,
        model_path=args.model_path,
        cam_num=args.cam_num,
    )
