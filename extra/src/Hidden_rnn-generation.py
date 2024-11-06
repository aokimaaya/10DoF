import argparse
import os
import time

import torch

from models.LSTM_6_working_incline import BASE

# from models.LSTM_6_working_C import BASE
# from models.LSTM_6 import BASE
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
# python -m src.Incline_motion_generation_lstm logs\learning_logs\wiping\20230110_140838-B_50_20_10x5x10x4_256 --step 512 --diff 2
# 20230109_085735-2023-01-08_22-11-06.071465-S_newx5x5x7 --step 1280 --diff 2
#
# 20230109_085751-2023-01-08_22-11-06.071465-S_newx5x5x7 --step 1280 --diff 2 --go_home 1(working but jumping)
#

# Big board
# python -m src.Incline_motion_generation_lstm logs\learning_logs\wiping\20230108_131440-2023-01-07_20-40-52.235052-B_45-2025qx5x5x6 --step 1280 --diff 2
# 20230108_131440-2023-01-07_20-40-52.235052-B_45-2025qx5x5x6 --step 1280 --diff 2 --go_home 1
#
# 20230105_113207-2023-01-04_23-16-08.814402-B_65-20x5x10x10 --step 1280 --diff 2(not working , the current intial values are messing up)
# 20230106_131225-2023-01-04_23-16-08.814402-B_65-20x5x10x10 --step 1280 --diff 2(working for 45 but 30 jerking -  test again)
#
# 20230107_140731-B_55-20x3x6x8_seq=256 --step 1280 --diff 2
#
# 20230105_215141-2023-01-04_23-16-08.814402-B_65-20x5x10x10 --step 1280 --diff 2 (not working)
#
# 20230105_224812-2023-01-04_23-16-08.814402-B_65-20x5x10x10_limseq=128 --step 128 --diff 2 (not working)
# 20230106_131225-2023-01-04_23-16-08.814402-B_65-20x5x10x10 --step 1280 --diff 2
# 20230105_113207-2023-01-04_23-16-08.814402-B_65-20x5x10x10 --step 1280 --diff 2
# 20230104_122634-2023-01-03_22-36-40.848797-B_55-20x3x6x8
#
# 20230103_004705-B_65-20x5x5x10 --step 1280 --diff 2
#
#
# 20221231_194822-B_65_20x5x5x10(7_3) --step 1280 --diff 2
# 20230104_013430-B_55-20x3x6x8-ran --step 768 --diff 2(retrain. not working)
# 20230104_030946-B_55-20x3x6x8-2-1 --step 768 --diff 8(not working)
# 20221231_185818-B_65-30x5x5x8_tt --step 1280 --diff 2
# 20230102_160439-Lstm_inc_B_65-20x5x5x10  --step 1280 --diff 2 (not working)

# 20221231_194822-B_65_20x5x5x10(7_3) --step 1280 --diff 2(working 45, notworking : 30)
# 20221230_185633-B_65-20x5x5x10_tt --step 1280 --diff 2
# 20221230_182922-B_65-20x5x5x10 --step 1280 --diff 2
# 20221228_190128-B_65-25x5_ --step 1280 --diff 2 (not working)
# 20221227_182144-B_Inc_ang-65-25x5x5x10 --step 1280 --diff 2
# 20221226_142732-65_20x5x5x10 --step 1280 --diff 2 (yes but multiples of 5 are not working -recheck)
# 20221222_225346-B_60-20x5x5-new --step 1280 --diff 2(okay but not great)
# 20221221_233407-B_60-20_5x5 --step 1280 --diff 2
# 20221207_202813-inc5x5x7 --step 1280 --diff 7
# 20221121_030630-LSTMauto256 --step 1536
# 20221121_030630-LSTMauto256 (working for incline)
#
# 20221108_200939
#
# python -m src.motion_generation logs\learning_logs\wiping\20221118_211323-xtra --cam_num 1
# 20221106_194323


def generate_motion(log_dir, step, diff, input_param, model_path, cam_num, go_home):
    try:
        # Lord Setting
        dataset = load_json(os.path.join(log_dir, "dataset.json"))
        ang_len = len(dataset["train_minmax"])
        DymControl = Control()
        getAngles = generateAngles(
            step * diff, 8, DymControl, cam_num
        )  # assign proper can number side view = 3 , top view = 1, inbuilt cam = 0
        logs_path = getAngles.create_logdir()
        print("Finished Loading settings")

        # Initialize Position
        nangle = 2048
        servo = "pro"
        DymControl.Enable(servo)
        # goal = dict(xm=[nangle, nangle, nangle], pro=DymControl.getj(servo))
        # change for every motion generation model
        low_pos6 = [205063, 24928, 2566]
        high_pos6 = [161746, 71000, 2587]

        low_pos5 = [205063, 24928, 2566]
        high_pos5 = [144924, 42391, 2587]

        low_pos4 = [205063, 24928, 2566]
        high_pos4 = [156184, 60496, 2587]

        low_pos3 = [205063, 24928, 2566]
        high_pos3 = [157657, 60792, -6142]

        low_pos2 = [205063, 24928, -3294]
        high_pos2 = [151315, 69759, -6142]

        low_pos1 = [205063, 24928, -3294]
        high_pos1 = [159010, 63306, -6142]

        low_pos0 = [205063, 24928, -3294]
        high_pos0 = [159926, 74826, 2587]

        low_pos = [213849, 43113, -6142]
        high_pos = [149437, 39938, 2587]
        hover_pos = [204995, 13147, 2566]
        Wipe_trj = [
            [high_pos0, low_pos],
            [high_pos1, low_pos],
            [high_pos2, low_pos],
            [high_pos3, low_pos],
            [high_pos4, low_pos],
            [high_pos5, low_pos],
            [high_pos6, low_pos],
        ]
        ###########################
        # low_pos=[213849,43113,-6142]
        # # high_pos=[149437,39938,2587]
        # hover_pos=[204995,13147,2566]
        # low_pos1=[203119,46603,-5522]
        # #high_pos1=[149437,39938,2587]

        # Wipe_inc_trj=[hover_pos,low_pos1]
        hoverpose_nt = [190744, 15348, -20686]
        low_pos_nt = [
            229676,
            25348,
            -20686,
        ]  # [223055,23788,-17985]#[213849,43113,-6142]
        Wipe_trj_nt = [hoverpose_nt, low_pos_nt]

        # DymControl.Initialize(Wipe_trj_nt)
        #############################
        # new incline

        B_hover_pos = [227311, -2708, 241532]
        B_mid_air = [205796, -12754, 241532]
        B_home = [180049, 23348, 6626]
        B_lowpos_nt = [
            213360,
            90492,
            247519,
        ]  # [205455, 94369, 247366]#   [200795, 95073, 247341]#[211100, 89935, 244369]
        # [214843, 75673, 241474]
        home_pos = [B_home, B_mid_air, B_hover_pos]
        return_pos = [B_hover_pos, B_mid_air, B_home]
        B_graze_pos = [B_lowpos_nt[0], B_lowpos_nt[1] - 20000, B_lowpos_nt[2]]
        if go_home:
            B_Wipe = [B_home, B_mid_air, B_graze_pos]
        else:
            B_Wipe = [B_hover_pos, B_lowpos_nt]
        # DymControl.Initialize(B_Wipe,0.008)
        goal = DymControl.goal
        #########################################################
        S_lowpos_nt = [
            169615,
            52854,
            400406,
        ]
        S_hover_pos = [169615, 16917, 400406]
        S_home_pos = [B_home, B_mid_air, S_hover_pos]
        S_return_pos = [S_hover_pos, B_mid_air, B_home]

        if go_home:
            S_Wipe = [B_home, B_mid_air, S_hover_pos, S_lowpos_nt]
        else:
            S_Wipe = [S_hover_pos, S_lowpos_nt]  # S_home_pos #
        # input_command = DymControl.Initialize(S_Wipe,0.008)

        # initialize low pass filter
        # d = 2
        # fs = 20
        # Wn = 2
        # DymControl.LiveLPF_initialize(d=d, fs=fs, Wn=Wn)

        # Prediction Setting
        model = BASE(ang_len)
        print("model_log:", model_path)
        model_path = log_dir + "/models/" + model_path + ".pth"
        model.load_state_dict(torch.load(model_path, map_location=torch.device("cpu")))
        # model.to("cpu")
        model.eval()
        print("Initialize.")
        print("Start online prediction at input_param={}".format(input_param))
        # if getch() == chr(0x1B):
        #     for servo in DymControl.servos:
        #         DymControl.Disable(servo)
        #     quit()

        # Start Online Prediction
        q = 1  # unit step
        cycles = 10
        train_rnn_hidden = None
        for t in range(step * diff):
            # if servo == "pro":
            # xm_align_angles=DymControl.rounderror(y_position,245.0,servo)
            # pro_filtered_values=DymControl.Filter(yposition[:3],"pro")
            # if  ((t) % (step) == 0):
            #     pro_current_values = DymControl.getj("pro")
            #     initial_pro_angles = DymControl.value2angle(
            #         pro_current_values, "pro"
            #     )
            #     if getch() != chr(0x20):
            #         for servo in DymControl.servos:
            #             DymControl.Disable(servo)
            #         getAngles.draw_results(logs_path, log_dir.split("\\")[-1], 1)
            #         quit()
            #     # print(
            #     #     "present_pro_angles",
            #     #     initial_pro_angles,
            #     #     DymControl.value2angle(yposition[:3], "pro"), B_lowpos_nt
            #     # )
            #     # DymControl.sin_movej(
            #     #     pro_current_values, yposition[:3], 0.01, servo
            #     # )
            #     Wipe_init = [pro_current_values,[B_lowpos_nt[0], B_lowpos_nt[1]-20000, B_lowpos_nt[2]],B_lowpos_nt]
            #     # Wipe_init = [pro_current_values,[S_lowpos_nt[0], S_lowpos_nt[1]-20000, S_lowpos_nt[2]],S_lowpos_nt]#

            #     # Wipe_init = [pro_current_values,[yposition[0], 64369, yposition[2]],yposition[:3]]

            #     # present_position=DymControl.sin_movej(Wipe_init[0],Wipe_init[1],020.,"pro")
            #     for w in range(len(Wipe_init) - 1):
            #         print(Wipe_init[w], Wipe_init[w + 1])
            #         present_position = DymControl.sin_movej(
            #             Wipe_init[w], Wipe_init[w + 1], 0.01, servo
            #         )
            # DymControl.sin_movej(
            #     B_lowpos_nt, yposition[:3], 0.01, servo
            # )
            # if t%q ==0:

            #     position= getAngles.record(servo, ang_len,0)
            #     print("actual angle: ",position[:3],"actual current: ",position[3:])
            # else:
            #     x_position, train_rnn_hidden = y_position, train_rnn_hidden
            position = getAngles.record(servo, ang_len, 0)
            print(t, "actual angle: ", position[:3], "actual current: ", position[3:])
            if t == 0:
                x_position = position
                # x_position[3:]=B_lowpos_nt#+[73,-4245,227]
                # x_image = image
                y_position, train_rnn_hidden = getAngles.prediction_lstm(
                    x_position, train_rnn_hidden, model, dataset, logs_path
                )

            else:
                x_position = input_param * position + (1 - input_param) * y_position
                y_position, train_rnn_hidden = getAngles.prediction_lstm(
                    x_position, train_rnn_hidden, model, dataset, logs_path
                )

            # Prediction

            # y_position, train_rnn_hidden = getAngles.prediction_lstm(
            #     x_position, train_rnn_hidden, model, dataset, logs_path
            # )

            # Visualize
            # getAngles.show_image(x_image, y_image_next)
            if t % q == 0:
                yposition, ycurrent = y_position[:ang_len], y_position[ang_len:]

                # Move Dym
                if servo == "xm":
                    xm_align_angles = DymControl.rounderror(yposition, 245.0, servo)

                    print("y_position", xm_align_angles, yposition, ycurrent)
                    print(
                        "XM angle",
                        DymControl.value2angle(xm_align_angles, "xm"),
                        "PRO angle",
                        DymControl.value2angle(yposition, "pro"),
                    )

                    # y_position[2]=0
                    # goal[servo]=y_position
                    # xm_align_angles[2]=2048
                    goal[servo] = xm_align_angles
                if servo == "pro":
                    # xm_align_angles=DymControl.rounderror(y_position,245.0,servo)
                    # pro_filtered_values=DymControl.Filter(yposition[:3],"pro")
                    if (t) % (step) == 0:  # /cycles
                        pro_current_values = DymControl.getj("pro")
                        initial_pro_angles = DymControl.value2angle(
                            pro_current_values, "pro"
                        )
                        if getch() != chr(0x20):
                            for servo in DymControl.servos:
                                DymControl.Disable(servo)
                            getAngles.draw_results(
                                logs_path, log_dir.split("\\")[-1], 1
                            )
                            quit()
                        print(
                            "present_pro_angles",
                            initial_pro_angles,
                            DymControl.value2angle(yposition[:3], "pro"),
                            yposition[:3],
                        )
                        DymControl.sin_movej(
                            pro_current_values, yposition[:3], 0.01, servo
                        )
                        print()
                        # Wipe_init = [pro_current_values,[yposition[0], 64369, yposition[2]],yposition[:3]]
                        # Wipe_init = [pro_current_values,[yposition[0], 64369, yposition[2]],yposition[:3]]

                        # present_position=self.sin_movej(Wipe_init[0],Wipe_init[1],020.,"pro")
                        # for w in range(len(Wipe_init) - 1):
                        #     print(Wipe_init[w], Wipe_init[w + 1])
                        #     present_position = DymControl.sin_movej(
                        #         Wipe_init[w], Wipe_init[w + 1], 0.01, servo
                        #     )
                        # DymControl.sin_movej(
                        #     B_lowpos_nt, yposition[:3], 0.01, servo
                        # )
                    xposition = []
                    for i in range(len(x_position)):
                        xposition.append(
                            normalize(x_position[i], [0, 1], dataset["train_minmax"][i])
                        )

                    # print("y_position",yposition,pro_filtered_values)#,xm_align_angles)
                    print(
                        "angle actual",
                        DymControl.value2angle(xposition[:3], "pro"),
                        xposition[:3],
                        "angle predicted",
                        DymControl.value2angle(yposition[:3], "pro"),
                        yposition[:3],
                    )
                    print(
                        "current actual",
                        xposition[3:],
                        "current predicted",
                        yposition[3:],
                    )  # "XM ",DymControl.value2angle(pro_filtered_values,"pro")

                    # y_position[2]=0
                    pro_filtered_values = yposition[:3]  #
                    goal[servo] = pro_filtered_values  # yposition[:3]#

                # DymControl.movej(goal, servo)
                if t % step == 0:
                    train_rnn_hidden = None
                    print("finish wipe cycle step")
                    print("Change angle", t)
                    if getch() != chr(0x20):
                        for servo in DymControl.servos:
                            DymControl.Disable(servo)
                        getAngles.draw_results(logs_path, log_dir.split("\\")[-1], 1)
                        quit()
                # time.sleep(0.05)

        # Draw Results
        # DymControl.Disable(servo)
        getAngles.draw_results(logs_path, log_dir.split("\\")[-1], 1)

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
    parser.add_argument("--diff", type=int, default=1)
    parser.add_argument("--go_home", type=int, default=0)
    args = parser.parse_args()
    print(args)
    generate_motion(
        log_dir=args.log_dir,
        step=args.step,
        input_param=args.input_param,
        model_path=args.model_path,
        cam_num=args.cam_num,
        diff=args.diff,
        go_home=args.go_home,
    )
