import datetime
import os
import random
import time

import numpy as np

from tool.dym_setup import Control
from utils.save_data import RecordData

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


# syntax:python -m src.test
DymControl = Control()
samples = 2048
cycles = 5
recordData = RecordData(samples * cycles, 8, DymControl)  #    "norecord"#

seq_num = 0
try:
    while True:
        input_command = input(
            """Input Desired State of control
                   i : initialize
                   r : record
                  LF : Leader follower Control
                   e : END
                   a : automate
                   u : uchida
               Input : """
        )  #
        # input(
        #     'First move to goal position then Press enter to start, "i" to initialize, "r" to read, "e" to finish. '
        # )
        # Change Initial Position
        if input_command == "u":

            ## Enable Servo Motors
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            pro_present_values = DymControl.getj("pro")
            print(pro_present_values, pro_present_values[:2])
            mx106_present_values = DymControl.rounderror(
                pro_present_values[:2], 245.0, "mx106"
            )
            DymControl.goal = dict(mx106=mx106_present_values, pro=pro_present_values)
            print(DymControl.goal)
            DymControl.movej(DymControl.goal, "mx106")

            print("Enter")
            if getch() == chr(0x1B):  # this is esc
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()
            DymControl.Disable("mx106")
            DymControl.Disable("xm430")

            DymControl.stats = 0
            # print("Started Leader Follower Control")
            try:
                i = 0
                while i < samples:

                    # DymControl.get_time(time.time(), i, "start")
                    # read from this
                    mx106_present_values = DymControl.getj("mx106")
                    mx106_present_values = mx106_present_values + [2048]
                    # xm540_current_values=DymControl.getj("xm540")

                    # print("xm540_current_values",xm540_current_values)
                    # DymControl.valuesforplot(time.time(),xm540_current_values,xm540_current_values,"xm540")

                    # #####
                    pro_target_values = DymControl.rounderror(
                        mx106_present_values, 245, "pro"
                    )  # DymControl.convertLF(xm540_angles,"pro")
                    # pro_filtered_values = DymControl.Filter(pro_target_values, "pro")
                    # DymControl.get_time(time.time(), i, "filter_pro_values")
                    # DymControl.valuesforplot(
                    #     time.time(), pro_target_values, pro_filtered_values, "pro"
                    # )
                    # DymControl.get_time(time.time(), i, "valuesforplot_pro")
                    DymControl.goal = dict(
                        mx106=mx106_present_values, pro=pro_target_values
                    )
                    print("Goal", DymControl.goal)
                    ############# move code important
                    DymControl.movej(DymControl.goal, "pro")
                    ############
                    # DymControl.get_time(time.time(), i, "Move_Follower")
                    print(mx106_present_values, pro_target_values)
                    print(
                        "mx106 ",
                        DymControl.value2angle(mx106_present_values, "xm540"),
                        "PRO",
                        DymControl.value2angle(pro_target_values, "pro"),
                    )
                    # DymControl.get_time(time.time(),i,"value2angle")

                    # print( str(i) +" ----------------------------")
                    if recordData != "norecord":
                        pass
                        # recordData.record(image_path, image_path_side, "pro")
                        # DymControl.get_time(time.time(), i, "record_data")
                    else:
                        # print("no record stats")
                        DymControl.stats = 1
                        # if DymControl.stat & (recordData.data_rows[recordData.time_step -1 ][2] <  DymControl.goal[servo][0]):
                        #     DymControl.stat = 0

                    # DymControl.get_time(time.time(), i, "finish")
                    if DymControl.stats:
                        i += 1
                    print(i)
                    ##wip code:
                    # for i in range(len(DymControl.DXL_IDs[servo])):
                    #     DymControl.toSigned32(
                    #         DymControl.angle_diff(DymControl.goal, present[i], i, servo), servo
                    #     )
                    # if stat & () :
                    #     i = i + 1
                    #     stat=1
                    # break

                # try:
                # #     xm540_present_values = DymControl.getj("xm540")
                # #     print(xm540_present_values)
                #     mx_present_values = DymControl.getj("mx106")
                #     print("mx_present_values",mx_present_values)
                #     DymControl.goal = dict(xm540=[2048,2048,2048],xm430=[2048,2048,2048],mx106=[1000,1000])#xm540_present_values)
                #     # DymControl.movej(DymControl.goal, "mx106")
                #     DymControl.movej(DymControl.goal, "xm540")
                #     # DymControl.movej(DymControl.goal, "xm430")
                #     i = 0
                #     while i < samples:
                #         # xm540_present_values = DymControl.getj("mx106")
                #         # print(xm540_present_values)
                #         mx_present_values = DymControl.getj("mx106")
                #         print("mx_present_values",mx_present_values)
                #         # DymControl.get_time(time.time(), i, "valuesforplot_pro")
                #         DymControl.goal = dict(xm540=[2048] + mx_present_values,mx106=[2048,2048])#xm540_present_values)
                #         DymControl.movej(DymControl.goal, "xm540")
                #         i+=1
            except:
                print(" error Interrupted")
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                input_command = ""

        elif input_command == "i":
            # input_command = DymControl.Initialize()
            input_command = ""
            ## Enable Servo Motors
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            try:
                goal = DymControl.goal
                print(goal)  # [132474,48933,-149]
                for servo in DymControl.servos:
                    if servo == "pro":

                        # while 1:
                        #     key1 = input("Input Goal Angle  for PROs ")
                        #     if key1 != "":
                        #         break
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

                        low_pos = [205063, 24928, -3294]  # [213849,43113,-6142]
                        high_pos = [149437, 39938, 2587]
                        hover_pos = [204995, 15147, 2566]
                        Wipe_trj = [
                            hover_pos,
                            low_pos,
                            high_pos0,
                        ]  # +[low_pos,high_pos0]*6#,low_pos,high_pos1]#,low_pos,high_pos2,low_pos,high_pos3,low_pos,high_pos4,low_pos,high_pos5,low_pos,high_pos6,low_pos]#
                        ########################################
                        # Wipe_big
                        B_highpos45 = [118892, 96144, 0]
                        B_lowpos45 = [205063, 24928, 0]
                        hover_pos = [204995, 13147, 2566]
                        Wipe_big = [hover_pos, B_lowpos45, B_highpos45, B_lowpos45]
                        # wipe_steel_bowl
                        # C_highpos_r=[189310,54382,-10947]
                        # C_highpos_l=[182539,49421,-4607]
                        # # C_highpos_m=[54382,96144,0]
                        # C_midpos_r=[193285,31537,-10947]
                        # C_midpos_l=[195089,30294,-4698]
                        # C_lowpos = [217022,30294,-10947]
                        # hover_posC=[217022,13147,-10947]
                        C_highpos_r = [169804, 33620, -16800]
                        C_highpos_l = [175560, 41183, 0]
                        # C_highpos_m=[54382,96144,0]

                        C_midpos_r = [190169, 25914, -16800]
                        C_midpos_l = [186655, 30294, 0]

                        C_lowpos_r = [207515, 26385, -16800]
                        C_lowpos_l = [208423, 22872, 0]

                        C_lowpos = [217022, 30294, -10947]
                        hover_posC = [217022, 13147, -10947]
                        # Wipe_C = DymControl.Wipe_Generator([[C_highpos_r,C_midpos_r,C_lowpos_r],[C_highpos_l,C_midpos_l,C_lowpos_l]],5)
                        # Wipe_C=[hover_posC,C_lowpos,C_midpos_r,C_highpos_r,C_midpos_r,C_lowpos,C_midpos_l,C_highpos_l,C_midpos_l,C_lowpos]
                        ########################################
                        # new table
                        high_pos_nt = {}
                        high_pos_nt[6] = [183680, 51405, -17985]
                        high_pos_nt[5] = [181768, 41433, -14061]
                        high_pos_nt[4] = [181108, 34407, -18710]
                        high_pos_nt[3] = [183189, 34407, -18710]
                        high_pos_nt[2] = [183189, 30596, -19868]
                        high_pos_nt[1] = [187656, 30596, -19868]
                        high_pos_nt[0] = [190744, 30596, -19868]
                        hoverpose_nt = [190744, 15348, -20686]
                        low_pos_nt = [
                            229676,
                            25348,
                            -20686,
                        ]  # [223055,23788,-17985]#[213849,43113,-6142]
                        Wipe_trj_nt = [hoverpose_nt, low_pos_nt]
                        for i in range(len(high_pos_nt.keys())):
                            Wipe_trj_nt.append(high_pos_nt[i])
                            Wipe_trj_nt.append(low_pos_nt)

                        # Wipe_trj_nt
                        # print(Wipe_C)
                        for ii, _pos in enumerate([Wipe_trj_nt]):  # [[242300,17500,0]]:
                            pro_current_values = DymControl.getj("pro")
                            initial_pro_angles = DymControl.value2angle(
                                pro_current_values, "pro"
                            )
                            print("present_pro_angles", initial_pro_angles)
                            run = 1
                            sin_step = 0.01
                            # if ii == 0:
                            #     Wipe_C_one=[pro_current_values]#,hover_posC,list(_pos[2]),list(_pos[1]),list(_pos[0]),list(_pos[1]),list(_pos[2])]
                            # else:
                            #     Wipe_C_one=[pro_current_values,list(_pos[2]),list(_pos[1]),list(_pos[0]),list(_pos[1]),list(_pos[2])]

                            Wipe_C_one = [pro_current_values] + _pos
                            print(ii, Wipe_C_one)
                            for w in range(len(Wipe_C_one) - 1):
                                print(Wipe_C_one[w], Wipe_C_one[w + 1])
                                present_position = DymControl.sin_movej(
                                    Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, servo
                                )
                                print("Enter", w / 2)
                                if w % 2 == 0:
                                    print("Change angle")
                                if getch() == chr(0x1B):
                                    for servo in DymControl.servos:
                                        DymControl.Disable(servo)
                                    quit()
                    else:
                        DymControl.movej(goal, servo)
                        run = 1
                        while run:
                            run = DymControl.reach(goal, servo)

                            print(
                                run,
                                DymControl.present_angles,
                                DymControl.value2angle(
                                    DymControl.present_angles, "xm540"
                                ),
                            )
                            break
            except KeyboardInterrupt:
                print("Interrupted")
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                input_command = ""
        # Automation
        elif input_command == "a":
            trails = 1
            # cycles= 2
            print("Initialize")
            low_pos1 = [205063, 24928, 2566]  # [213849,43113,-6142]#[205063,24928,2566]
            high_pos1 = [149437, 39938, 2587]
            hover_pos = [204995, 13147, 2566]
            Wipe_trj = [hover_pos, low_pos1]
            ###########
            C_lowpos = [217022, 30294, -10947]
            hover_posC = [217022, 13147, -10947]
            #####################new table
            hoverpose_nt = [190744, 15348, -20686]
            low_pos_nt = [
                229676,
                25348,
                -20686,
            ]  # [223055,23788,-17985]#[213849,43113,-6142]
            Wipe_trj_nt = [hoverpose_nt, low_pos_nt]
            input_command = DymControl.Initialize(Wipe_trj_nt)
            print("Enter for automation")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()
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
            C_highpos_r = [169804, 33620, -16800]
            C_highpos_l = [175560, 41183, 0]
            # C_highpos_m=[54382,96144,0]

            C_midpos_r = [190169, 25914, -16800]
            C_midpos_l = [186655, 30294, 0]

            C_lowpos_r = [207515, 26385, -16800]
            C_lowpos_l = [208423, 22872, 0]
            hover_posC = [217022, 13147, -10947]
            #################################
            high_pos_nt = {}
            high_pos_nt[6] = [183680, 51405, -17985]
            high_pos_nt[5] = [181768, 41433, -14061]
            high_pos_nt[4] = [181108, 34407, -18710]
            high_pos_nt[3] = [183189, 34407, -18710]
            high_pos_nt[2] = [183189, 30596, -19868]
            high_pos_nt[1] = [187656, 30596, -19868]
            high_pos_nt[0] = [190744, 30596, -19868]
            hoverpose_nt = [190744, 15348, -20686]
            low_pos_nt = [
                229676,
                25348,
                -20686,
            ]  # [223055,23788,-17985]#[213849,43113,-6142]
            Wipe_trj_nt = []
            for i in range(len(high_pos_nt.keys())):
                Wipe_trj_nt.append([high_pos_nt[i], low_pos_nt])
                # Wipe_trj_nt.append(low_pos_nt)
            # Wipe_C = DymControl.Wipe_Generator([[C_highpos_r,C_midpos_r,C_lowpos_r],[C_highpos_l,C_midpos_l,C_lowpos_l]],10)
            # Wipe_C = DymControl.Wipe_Generator([[C_highpos_r,C_lowpos_r],[C_highpos_l,C_lowpos_l]],10)
            print(Wipe_trj_nt)
            for ii, _pos in enumerate(Wipe_trj_nt):  # [[242300,17500,0]]:
                # pro_current_values=DymControl.getj("pro")
                # initial_pro_angles=DymControl.value2angle(pro_current_values,"pro")
                # print("present_pro_angles",initial_pro_angles)
                # if ii == 0:
                #     Wipe_init = [pro_current_values,list(_pos[2])]
                #     present_position=DymControl.sin_movej(Wipe_init[0],Wipe_init[1],0.02,"pro")
                # Wipe_C_one=[list(_pos[2]),list(_pos[1]),list(_pos[0]),list(_pos[1]),list(_pos[2])] # with midpos
                Wipe_C_one = [
                    list(_pos[1]),
                    list(_pos[0]),
                    list(_pos[1]),
                ]  # direct end to end
                # break
                comments = 0
                seq_num = DymControl.Automation_data_C(
                    int(seq_num), recordData, samples, cycles, trails, Wipe_C_one
                )
                # print(ii,"angle")
                print("Change angle", ii)
                print("Enter for next")
                if getch() == chr(0x1B):
                    for servo in DymControl.servos:
                        DymControl.Disable(servo)
                    quit()
            input_command = DymControl.Initialize([hoverpose_nt])

            print("Automation_Complete")

        # Start Data Collection
        elif input_command == "r":
            low_pos1 = [205063, 24928, 2566]  # [213849,43113,-6142]#
            high_pos1 = [149437, 39938, 2587]
            hover_pos = [204995, 13147, 2566]
            Wipe_trj = [hover_pos, low_pos1]
            #####
            B_highpos45 = [118892, 96144, 0]
            B_lowpos45 = [205063, 24928, 0]
            hover_pos = [204995, 13147, 2566]
            Wipe_big = [hover_pos, B_lowpos45]  # ,B_highpos45,B_lowpos45]
            ###
            C_highpos_r = [189310, 54382, -10947]
            # C_highpos_m=[54382,96144,0]
            C_midpos = [193285, 31537, -10947]

            C_lowpos = [217022, 30294, -10947]
            hover_posC = [204995, 13147, 2566]
            Wipe_C = [hover_posC, C_lowpos]  # ,B_highpos45,B_lowpos45]
            input_command = DymControl.Initialize(Wipe_trj)
            print("Enter")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()

            DymControl.LeaderFollowerRecord(seq_num, recordData, samples)

            seq_num += 1
            input_command = ""

        elif input_command == "LF":
            low_pos1 = [205063, 24928, 2566]  # [213849,43113,-6142]#
            high_pos1 = [149437, 39938, 2587]
            hover_pos = [204995, 13147, 2566]
            Wipe_trj = [hover_pos, low_pos1]
            input_command = DymControl.Initialize(Wipe_trj)
            print("Enter")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()

            DymControl.LeaderFollowerRecord(seq_num, "norecord", samples)
            input_command = ""
        elif input_command == "e":
            for servo in DymControl.servos:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()

except KeyboardInterrupt:
    print("Interrupted")


finally:
    print("Finish Data Collection.")


# if __name__ == "main":
#     data_collection()
