import os

from tool.dym_setup import Control
# from utils.save_data import RecordData

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


DymControl = Control()
samples = 3000
cycles = 1
trails = 1
recordData = "norecord"#RecordData(samples*cycles, 8, DymControl)#"norecord"  # 
seq_num = 38
try:
    while True:
        input_command = input(
            """Input Desired State of control
            old_i : initialize
                r : record
                LF: Leader follower Control
                e : END                   
                a : automate
                u : 6DOF LF
                g : Wipe Sin motion
                i : initialize for hbk dual arm
              LF6 : LF 6DOF new
                h : go back to home position
                e1: END for F1
                e2: END for F2

            Input : """
        )  #
        # input(
        #     'First move to goal position then Press enter to start, "i" to initialize, "r" to read, "e" to finish. '
        # )
        # Change Initial Position

        if input_command == "g":
            # input_command = DymControl.Initialize()
            input_command = ""
            ## Enable Servo Motors
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            try:
                pro_current_values = {}
                goal = DymControl.goal
                print(goal)  # [132474,48933,-149]
                # for servo in DymControl.servos:
                #     if servo in DymControl.servo_com["L"]:

                # new table
                high_pos_nt = {}
                # high_pos_nt[6]=[183680,51405,-17985]
                # high_pos_nt[5]=[181768,41433,-14061]
                # high_pos_nt[4]=[181108,34407,-18710]

                # #table leader mx106 & xm430F itoen matcha botle 90 pick and place
                # high_pos_nt[5]=dict(mx106=[1990,2530],xm430x=[2807,2048,1538,1928,1793])
                # high_pos_nt[4]= dict(mx106=[1990,2711],xm430F=[2751,2048,1538,1928,1793])
                # high_pos_nt[3]=dict(mx106=[1990,2549],xm430F=[2751,2048,1599,1928,2000])
                # high_pos_nt[2]=dict(mx106=[2967,2549],xm430F=[2751,2048,1599,1928,2000])
                # high_pos_nt[1]=dict(mx106=[2967,2696],xm430F=[2751,2048,1599,1928,2000])
                # high_pos_nt[0]=dict(mx106=[2968,2530],xm430F=[2953,2048,1638,1928,1773])

                # Small sinusoidal motion
                # high_pos_nt[0]=dict(pro= [162953,1420,148151],xm540=[550,2101,1909],xm430F=[1589])

                # high_pos_nt[5]=dict(pro= [429290,-31678,84570],xm540=[550,2128,2229],xm430F=[1821])
                high_pos_nt[7] = dict(
                    pro=[434195, -6853, 120888], xm540=[550, 2101, 1909], xm430F=[1950]
                )

                high_pos_nt[6] = dict(
                    pro=[434195, -6853, 159093], xm540=[550, 2101, 1909], xm430F=[1950]
                )

                high_pos_nt[5] = dict(
                    pro=[434195, -6853, 159093], xm540=[550, 2101, 1909], xm430F=[2500]
                )

                high_pos_nt[4] = dict(
                    pro=[434195, -12532, 111086], xm540=[550, 2101, 1909], xm430F=[2500]
                )
                high_pos_nt[3] = dict(
                    pro=[162953, 1420, 111086], xm540=[550, 2101, 1909], xm430F=[2500]
                )

                # high_pos_nt[1]=dict(pro= [179339,-22182,120888],xm540=[550,2128,2229],xm430F=[1993])
                high_pos_nt[2] = dict(
                    pro=[162953, 1420, 190551], xm540=[550, 2101, 1909], xm430F=[2500]
                )

                high_pos_nt[1] = dict(
                    pro=[162953, 1420, 149051], xm540=[550, 2101, 1909], xm430F=[1950]
                )

                high_pos_nt[0] = dict(
                    pro=[162953, 1420, 120888], xm540=[550, 2101, 1909], xm430F=[1950]
                )

                #####################################################################################
                # Big sinusoidal motion
                high_pos_B = {}
                # high_pos_nt[0]=dict(pro= [162953,1420,148151],xm540=[550,2101,1909],xm430F=[1589])

                # high_pos_nt[5]=dict(pro= [429290,-31678,84570],xm540=[550,2128,2229],xm430F=[1821])
                # high_pos_B[7]=dict(pro= [434195,-6853,120888],xm540=[550,2101,1909],xm430F=[1950])

                # high_pos_B[6]=dict(pro= [434195,-6853,159093],xm540=[550,2101,1909],xm430F=[1950])

                # high_pos_B[5]=dict(pro= [434195,-6853,159093],xm540=[550,2101,1909],xm430F=[2500])

                # high_pos_B[4]=dict(pro= [434195,-12532,111086],xm540=[550,2101,1909],xm430F=[2500])
                # high_pos_B[5]=dict(pro= [-401092,34520,-184336],xm540=[3460,1275,1063],xm430F=[1806])

                high_pos_B[7] = dict(
                    pro=[-420677, 30862, -221261],
                    xm540=[1891, 2723, 2535],
                    xm430F=[1500],
                )

                high_pos_B[6] = dict(
                    pro=[-420677, 35862, -221261],
                    xm540=[1891, 2723, 2535],
                    xm430F=[1500],
                )

                high_pos_B[5] = dict(
                    pro=[-420677, 35862, -221261],
                    xm540=[1891, 2723, 2535],
                    xm430F=[2015],
                )
                xm_430_7 = 2015
                high_pos_B[4] = dict(
                    pro=[-351432, 32963, -88419],
                    xm540=[1891, 2723, 2535],
                    xm430F=[xm_430_7],
                )  # [xm540_4,xm540_5,xm540_6]
                xm540_4 = 2408
                xm540_5 = 2272
                xm540_6 = 2245

                high_pos_B[3] = dict(
                    pro=[-115856, 32605, -83601],
                    xm540=[xm540_4, xm540_5, xm540_6],
                    xm430F=[xm_430_7],
                )
                pro_3 = -183659
                high_pos_B[2] = dict(
                    pro=[-115856, 21117, pro_3],
                    xm540=[xm540_4, xm540_5, xm540_6],
                    xm430F=[xm_430_7],
                )

                high_pos_B[1] = dict(
                    pro=[-115856, 21117, pro_3],
                    xm540=[xm540_4, xm540_5, xm540_6],
                    xm430F=[1697],
                )

                high_pos_B[0] = dict(
                    pro=[-115856, 32963, -88419],
                    xm540=[xm540_4, xm540_5, xm540_6],
                    xm430F=[1697],
                )

                # high_pos_B[0]=dict(pro= [-401092,26383,-136865],xm540=[4699,2715,1830],xm430F=[1697]
                ####################################################################################################
                # angle index dictionary
                ang_index = {}
                for i in range(10):
                    ang_index[65 - 5 * i] = i
                # Wipe_big_latest 12/20
                B_highpos = {}
                B_lowpos = {}
                B_lowpos_u = {}
                B_lowpos_l = {}
                B_midpos = {}
                B_midpos_u = {}
                B_midpos_l = {}
                # 20
                B_lowpos_u[9] = [213360, 90492, 247519]
                B_midpos_u[9] = [147858, 128083, 247519]
                B_highpos[9] = [0, 212174, 253045]
                B_midpos_l[9] = [120507, 126015, 254592]
                B_lowpos_l[9] = [217771, 93144, 247331]
                # 25
                B_lowpos_u[8] = [205455, 94369, 247366]
                B_midpos_u[8] = [146622, 111502, 247366]
                B_highpos[8] = [0, 200758, 254625]
                B_midpos_l[8] = [151030, 118198, 244369]
                B_lowpos_l[8] = [211100, 89935, 244369]
                # 30
                B_lowpos_u[7] = [206261, 84643, 247341]
                B_midpos_u[7] = [153730, 106682, 247341]
                B_highpos[7] = [0, 161119, 257265]
                B_midpos_l[7] = [153730, 106682, 247341]
                B_lowpos_l[7] = [200795, 95073, 247341]
                # 35
                B_lowpos_u[6] = [206597, 87247, 242295]
                B_midpos_u[6] = [132242, 103805, 250222]
                B_highpos[6] = [9928, 172387, 253180]
                B_midpos_l[6] = [143049, 108290, 253180]
                B_lowpos_l[6] = [201387, 84319, 247298]
                # 42 #39
                B_lowpos_u[5] = [206200, 85113, 247298]
                B_midpos_u[5] = [143501, 91180, 251688]
                B_highpos[5] = [34170, 132517, 250242]
                B_midpos_l[5] = [134205, 92303, 250242]
                B_lowpos_l[5] = [197933, 85241, 242270]
                # 47   #45
                B_lowpos_u[4] = [208484, 82117, 243249]
                B_midpos_u[4] = [146497, 89136, 247331]
                B_highpos[4] = [47475, 122678, 250235]
                B_midpos_l[4] = [134205, 90303, 250242]
                B_lowpos_l[4] = [214843, 84023, 243225]  # [217933, 85241, 242270]
                # 50
                B_lowpos_u[3] = [214843, 86023, 243225]
                B_midpos_u[3] = [151503, 84057, 243225]
                B_highpos[3] = [47475, 122678, 250235]
                B_midpos_l[3] = [156425, 82057, 243225]
                B_lowpos_l[3] = [214843, 85023, 243225]
                # 55
                B_lowpos_l[2] = [210407, 86463, 243191]
                B_midpos_l[2] = [151030, 87746, 250222]
                B_highpos[2] = [67983, 102667, 250222]
                B_midpos_u[2] = [156009, 77535, 241781]
                B_lowpos_u[2] = [203849, 84457, 241781]
                # 60
                B_lowpos_u[1] = [213917, 76202, 241533]
                B_midpos_u[1] = [182622, 72688, 241503]  # [158795,68884,241533]
                B_highpos[1] = [95073, 86990, 250248]
                B_midpos_l[1] = [159871, 75008, 246264]
                B_lowpos_l[1] = [218166, 88523, 246264]
                # 65
                B_highpos[0] = [124120, 61735, 246968]  # [92057,46973,251730]#
                B_midpos_u[0] = [188622, 62688, 241503]  # [167016,39314,241481]#
                B_midpos_l[0] = [178622, 75688, 241503]  # [167016,39314,241481]#
                B_lowpos_u[0] = [
                    219166,
                    77523,
                    241503,
                ]  # [233909,55836,241532]#[227592,59564,241532]
                B_lowpos_l[0] = [219166, 80523, 241503]

                ###############################
                # B_highpos[0] = dict(
                #     pro=[254404, -99595, -53315]
                # )  # [92057,46973,251730]#
                # # B_midpos_u[0] = dict(pro= [124120, 61735, 246968])#[167016,39314,241481]#
                # # B_midpos_l[0] = [178622, 75688, 241503]#[167016,39314,241481]#
                # B_lowpos_u[0] = dict(
                #     pro=[248931, -74990, -206015]
                # )
                # B_hover_pos = dict(pro=[248931, -47690, -164502])
                # B_graze_pos = dict(pro=[248931, -70834, -206015])#[208484, 82117, 243249]
                B_mid_air = dict(pro=[170173, -8591, -243392])
                # B_home = dict(pro=[-20862, -10591, -243392]) # [233909,55836,241532]#[227592,59564,241532]
                # home_pos = [B_home, B_mid_air, B_hover_pos, B_graze_pos]
                # return_pos = [B_hover_pos, B_mid_air, B_home]
                B_graze_pos = dict(pro=[243225, 70023, 214843])

                BWipe_trj_nt = []
                fix_angle = ""
                while 1:
                    fix_angle = input(
                        """Input fixed angle
                        Fixed angle : """
                    )
                    if fix_angle != "":
                        break
                for i in range(len(B_highpos.keys())):
                    # if i ==:#in [3,7,9]
                    i = ang_index[int(fix_angle)]
                    # print(i)
                    # if  i > 2:
                    B_Wipe = [B_graze_pos]
                    B_Wipe.append(dict(pro=B_lowpos_u[9][::-1]))

                    B_Wipe.append(dict(pro=B_highpos[i][::-1]))
                    B_Wipe.append(dict(pro=B_lowpos_u[9][::-1]))
                    print("B_Wipe", B_Wipe)
                    BWipe_trj_nt.append(B_Wipe)
                    # BWipe_trj_nt.append(return_pos)
                    # Wipe_trj_nt
                    # print(Wipe_C)
                # Usage example:
                save_folder = r"C:\Users\81809\Desktop\Three_DoF_Robotarm-main\Three_DoF_Robotarm\utils\fig\frames__{}".format(
                    fix_angle
                )

                os.makedirs(save_folder, exist_ok=True)
                excel_file = save_folder + r"\frame_data_{}.xlsx".format(fix_angle)

                # aruco_detector.capture_frames(save_folder, 'start', angle=45)  # Capture "before" frames with angle 45

                Inv = "0"
                for ii, _pos in enumerate(BWipe_trj_nt):  # [[242300,17500,0]]:
                    for servo in DymControl.servo_com["F"]:
                        pro_current_values[servo] = DymControl.getj(servo)
                    # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                    initial_pro_angles = pro_current_values  # DymControl.value2angle(pro_current_values,"pro")
                    print("present_pro_angles", initial_pro_angles)
                    run = 1
                    sin_step = 0.02
                    # if ii == 0:
                    #     Wipe_C_one=[pro_current_values]#,hover_posC,list(_pos[2]),list(_pos[1]),list(_pos[0]),list(_pos[1]),list(_pos[2])]
                    # else:
                    #     Wipe_C_one=[pro_current_values,list(_pos[2]),list(_pos[1]),list(_pos[0]),list(_pos[1]),list(_pos[2])]
                    # while 1:
                    #     Inv = input("Input 1 for return ")
                    #     if Inv != "":
                    #         break
                    Inv = "0"
                    slope = ""
                    while 1:
                        slope = input(
                            """Input Slope angle
                            Slope : """
                        )
                        if slope != "":
                            break
                    Wipe_C_one = [pro_current_values] + _pos
                    print(ii, Wipe_C_one)
                    if Inv == "0":

                        for w in range(len(Wipe_C_one) - 1):
                            # print("Wipe_C_one[w]",Wipe_C_one[w],"Wipe_C_one[w+1]",Wipe_C_one[w+1])
                            # print("spacebar for next")
                            present_position = DymControl.sin_movej(
                                Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, "F"
                            )
                            print("Enter", w)
                            # if w % 2 == 0:
                            print("Change angle")

                            # if w == 1:
                            #     # aruco_detector.capture_frames(save_folder, 'start', angle=slope)  # Capture "before" frames with angle 45
                            # if w == 3:
                            #     # aruco_detector.capture_frames(save_folder, 'end', angle=slope)  # Capture "before" frames with angle 45
                            #     # aruco_detector.save_data_to_excel(excel_file)
                            if getch() == chr(0x1B):
                                # for servo in DymControl.servos:
                                #     DymControl.Disable(servo)
                                quit()
                        Inv = ""
                # while 1:
                #     Inv = input("Input 1 for return ")
                #     if Inv != "":
                #         break

                if Inv == "2":
                    for servo in DymControl.servo_com["F"]:
                        pro_current_values[servo] = DymControl.getj(servo)
                    # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                    Wipe_C_one = Wipe_trj_nt + [pro_current_values]
                    Wlen = len(Wipe_C_one) - 1
                    for w in range(Wlen):
                        w = Wlen - w - 1
                        print(Wipe_C_one[w], Wipe_C_one[w + 1])
                        present_position = DymControl.sin_movej(
                            Wipe_C_one[w + 1], Wipe_C_one[w], sin_step, "F"
                        )
                        print("Enter", w / 2)
                        if w % 2 == 0:
                            print("Change angle")
                        # if getch() == chr(0x1b):
                        #     # for servo in DymControl.servos:
                        #     #     DymControl.Disable(servo)
                        #     quit()
                    Inv = ""

            except KeyboardInterrupt:
                print("Interrupted")
                # for servo in DymControl.servos:
                #     DymControl.Disable(servo)
                input_command = ""
        # Automation
        elif input_command == "u":

            DymControl.goal = dict(
                xm540=[2048, 2048, 2048],
                pro=[0, 0, 0],
                mx106=[2048, 2048],
                xm430=[2048, 2048, 2048, 2048],
            )
            ## Enable Servo Motors
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            # DymControl.movej(DymControl.goal, "xm540")

            pro_present_values = DymControl.signed(DymControl.getj("pro"), "pro")
            xm540_present_values = DymControl.getj("xm540")
            print(
                "pro_present_values",
                pro_present_values,
                "xm540_present_values",
                xm540_present_values,
            )
            mx106_present_values = DymControl.rounderror(
                pro_present_values[:2], 245.0, "mx106"
            )
            xm430_present_values = DymControl.rounderror(
                [pro_present_values[2]] + xm540_present_values, 245.0, "xm430"
            )
            DymControl.goal["mx106"] = mx106_present_values
            DymControl.goal["pro"] = pro_present_values
            DymControl.goal["xm430"] = xm430_present_values
            DymControl.goal["xm540"] = xm540_present_values
            print(DymControl.goal)
            DymControl.movej(DymControl.goal, "mx106")
            DymControl.movej(DymControl.goal, "xm430")

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
                    xm430_present_values = DymControl.getj("xm430")
                    mx106_present_values = mx106_present_values + [
                        xm430_present_values[0]
                    ]
                    # xm540_current_values=DymControl.getj("xm540")

                    # print("xm540_current_values",xm540_current_values)
                    # DymControl.valuesforplot(time.time(),xm540_current_values,xm540_current_values,"xm540")

                    # #####
                    pro_target_values = DymControl.rounderror(
                        mx106_present_values, 245, "pro"
                    )
                    xm540_target_values = xm430_present_values[1:]
                    # pro_filtered_values = DymControl.Filter(pro_target_values, "pro")
                    # DymControl.get_time(time.time(), i, "filter_pro_values")
                    # DymControl.valuesforplot(
                    #     time.time(), pro_target_values, pro_filtered_values, "pro"
                    # )
                    # DymControl.get_time(time.time(), i, "valuesforplot_pro")
                    # DymControl.goal = dict(mx106=mx106_present_values, pro=pro_target_values)
                    # Read follower
                    DymControl.goal["mx106"] = mx106_present_values
                    DymControl.goal["xm430"] = xm430_present_values

                    # target leader
                    DymControl.goal["pro"] = pro_target_values
                    DymControl.goal["xm540"] = xm540_target_values

                    print("Goal", DymControl.goal)
                    ############# move code important
                    DymControl.movej(DymControl.goal, "pro")
                    DymControl.movej(DymControl.goal, "xm540")
                    ############
                    # DymControl.get_time(time.time(), i, "Move_Follower")
                    # print(mx106_present_values,pro_target_values)
                    print(
                        "mx106 ",
                        DymControl.value2angle(mx106_present_values, "xm540"),
                        "xm430",
                        DymControl.value2angle(xm430_present_values, "xm430"),
                    )
                    print(
                        "PRO",
                        DymControl.value2angle(pro_target_values, "pro"),
                        "xm540 ",
                        DymControl.value2angle(xm540_target_values, "xm540"),
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
                #     DymControl.goal = dict(xm540=[2048,2048,2048],xm430F=[2048,2048,2048],mx106=[1000,1000])#xm540_present_values)
                #     # DymControl.movej(DymControl.goal, "mx106")
                #     DymControl.movej(DymControl.goal, "xm540")
                #     # DymControl.movej(DymControl.goal, "xm430F")
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
                # for servo in DymControl.servos:
                #     DymControl.Disable(servo)
                input_command = ""

        # Change Initial Position
        if input_command == "old_i":
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
                        """format of the angles list
                        [servo1 , servo2 , servo3]
                        The values are manually collected from the Dynamixel Wizard
                        Wiping action starts at Low postion (low pos) to high position (high_pos) to Low position (low_pos)
                        The above completes one cycle.
                        - C = Curved
                        - B = Big
                        - nt = new table
                        """
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
                        ]
                        ########################################

                        C_highpos_r = [169804, 33620, -16800]
                        C_highpos_l = [175560, 41183, 0]

                        C_midpos_r = [190169, 25914, -16800]
                        C_midpos_l = [186655, 30294, 0]

                        C_lowpos_r = [207515, 26385, -16800]
                        C_lowpos_l = [208423, 22872, 0]

                        C_lowpos = [217022, 30294, -10947]
                        hover_posC = [217022, 13147, -10947]
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
                        ]
                        # this is the wipe list
                        Wipe_trj_nt = [
                            hoverpose_nt,
                            low_pos_nt,
                            high_pos_nt[0],
                            low_pos_nt,
                            high_pos_nt[6],
                            low_pos_nt,
                        ]
                        ########################################
                        # new table curve
                        ## multi start postition
                        C_highpos_r_nt = [169804, 33620, -16800]
                        C_highpos_l_nt = [175560, 41183, 0]
                        # C_highpos_m=[54382,96144,0]

                        C_midpos_r_nt = [206498, 36118, -31848]
                        C_midpos_l_nt = [186655, 30294, 0]

                        C_lowpos_r_nt = [226340, 43803, -31848]
                        C_lowpos_l_nt = [208423, 22872, 0]

                        C_lowpos_nt = [217022, 30294, -10947]
                        hover_posC_nt = [217022, 13147, -10947]
                        CWipe_trj_nt = []
                        for i in range(len(high_pos_nt.keys())):
                            CWipe_trj_nt.append(high_pos_nt[i])
                            CWipe_trj_nt.append(low_pos_nt)
                            #################################
                        Chigh_pos_nt = {}
                        Chigh_pos_nt[5] = [202836, 54609, -27510]  # 9
                        Chigh_pos_nt[4] = [205095, 53190, -29945]  # 8
                        Chigh_pos_nt[3] = [210826, 51405, -32643]  # 7
                        Chigh_pos_nt[2] = [202161, 52733, -33151]  # 3
                        Chigh_pos_nt[1] = [222480, 53142, -37998]  # 2
                        Chigh_pos_nt[0] = [209929, 52853, -33940]  # 0
                        Cmid_pos_nt = {}

                        Cmid_pos_nt[5] = [226681, 36443, -34491]  # 9
                        Cmid_pos_nt[4] = [224347, 30199, -36671]  # 8
                        Cmid_pos_nt[3] = [221969, 36760, -35087]  # 7
                        Cmid_pos_nt[2] = [222587, 34200, -36843]  # 3
                        Cmid_pos_nt[1] = [233131, 31260, -37998]  # 2
                        Cmid_pos_nt[0] = [222671, 29998, -37998]  # 0
                        Choverpose_nt = [244482, 15348, -37998]
                        Clow_pos_nt1 = [244482, 36760, -37998]
                        Clow_pos_nt = [
                            236860,
                            27049,
                            -37997,
                        ]  # [223055,23788,-17985]#[213849,43113,-6142]
                        CWipe_trj_nt = []
                        ############################################
                        # Wipe_big_latest 12/20
                        B_highpos = {}
                        B_lowpos = {}
                        B_lowpos_u = {}
                        B_lowpos_l = {}
                        B_midpos = {}
                        B_midpos_u = {}
                        B_midpos_l = {}

                        B_lowpos_u[9] = [213360, 90492, 247519]
                        B_midpos_u[9] = [147858, 128083, 247519]
                        B_highpos[9] = [0, 212174, 253045]
                        B_midpos_l[9] = [120507, 126015, 254592]
                        B_lowpos_l[9] = [217771, 93144, 247331]
                        # 25
                        B_lowpos_u[8] = [205455, 94369, 247366]
                        B_midpos_u[8] = [146622, 111502, 247366]
                        B_highpos[8] = [0, 200758, 254625]
                        B_midpos_l[8] = [151030, 118198, 244369]
                        B_lowpos_l[8] = [211100, 89935, 244369]
                        # 30
                        B_lowpos_u[7] = [206261, 84643, 247341]
                        B_midpos_u[7] = [153730, 106682, 247341]
                        B_highpos[7] = [0, 161119, 257265]
                        B_midpos_l[7] = [153730, 106682, 247341]
                        B_lowpos_l[7] = [200795, 95073, 247341]
                        # 35
                        B_lowpos_u[6] = [206597, 87247, 242295]
                        B_midpos_u[6] = [132242, 103805, 250222]
                        B_highpos[6] = [9928, 172387, 253180]
                        B_midpos_l[6] = [143049, 108290, 253180]
                        B_lowpos_l[6] = [201387, 84319, 247298]
                        # 42 #39
                        B_lowpos_u[5] = [206200, 85113, 247298]
                        B_midpos_u[5] = [143501, 91180, 251688]
                        B_highpos[5] = [34170, 132517, 250242]
                        B_midpos_l[5] = [134205, 92303, 250242]
                        B_lowpos_l[5] = [197933, 85241, 242270]
                        # 47   #45
                        B_lowpos_u[4] = [208484, 82117, 243249]
                        B_midpos_u[4] = [146497, 89136, 247331]
                        B_highpos[4] = [47475, 122678, 250235]
                        B_midpos_l[4] = [134205, 90303, 250242]
                        B_lowpos_l[4] = [
                            214843,
                            84023,
                            243225,
                        ]  # [217933, 85241, 242270]
                        # 50
                        B_lowpos_u[3] = [214843, 86023, 243225]
                        B_midpos_u[3] = [151503, 84057, 243225]
                        B_highpos[3] = [47475, 122678, 250235]
                        B_midpos_l[3] = [156425, 82057, 243225]
                        B_lowpos_l[3] = [214843, 85023, 243225]
                        # 55
                        B_lowpos_l[2] = [210407, 86463, 243191]
                        B_midpos_l[2] = [151030, 87746, 250222]
                        B_highpos[2] = [67983, 102667, 250222]
                        B_midpos_u[2] = [156009, 77535, 241781]
                        B_lowpos_u[2] = [203849, 84457, 241781]
                        # 60
                        B_lowpos_u[1] = [213917, 76202, 241533]
                        B_midpos_u[1] = [182622, 72688, 241503]  # [158795,68884,241533]
                        B_highpos[1] = [95073, 86990, 250248]
                        B_midpos_l[1] = [159871, 75008, 246264]
                        B_lowpos_l[1] = [218166, 88523, 246264]
                        # 65
                        B_highpos[0] = [124120, 61735, 246968]  # [92057,46973,251730]#
                        B_midpos_u[0] = [
                            188622,
                            62688,
                            241503,
                        ]  # [167016,39314,241481]#
                        B_midpos_l[0] = [
                            178622,
                            75688,
                            241503,
                        ]  # [167016,39314,241481]#
                        B_lowpos_u[0] = [
                            219166,
                            77523,
                            241503,
                        ]  # [233909,55836,241532]#[227592,59564,241532]
                        B_lowpos_l[0] = [219166, 80523, 241503]

                        B_hover_pos = [227311, -2708, 241532]
                        B_graze_pos = [213360, 50492, 247519]
                        B_mid_air = [205796, -12754, 241532]
                        B_home = [180049, 23348, 6626]
                        home_pos = [B_home, B_mid_air, B_hover_pos]
                        return_pos = [B_hover_pos, B_mid_air, B_home]
                        B_Wipe = [
                            B_graze_pos,
                            B_lowpos_u[9],
                        ]  # home_pos#[B_graze_pos,B_lowpos_u[9]]#
                        ###########################################################
                        # On white self small board
                        ########
                        # new table
                        high_pos_nt = {}
                        high_pos_nt[6] = [119014, 45074, 400406]
                        high_pos_nt[5] = [113981, 48517, 400406]
                        high_pos_nt[4] = [112525, 51178, 400406]
                        high_pos_nt[3] = [108975, 53963, 400406]
                        high_pos_nt[2] = [107801, 55063, 400406]
                        high_pos_nt[1] = [104405, 57233, 400406]
                        high_pos_nt[0] = [102951, 71129, 400406]
                        S_hover_pos = [169615, 16917, 400406]
                        low_pos_nt = [
                            169615,
                            52854,
                            400406,
                        ]  # [223055,23788,-17985]#[213849,43113,-6142]
                        S_home_pos = [B_home, B_mid_air, S_hover_pos]
                        S_return_pos = [S_hover_pos, B_mid_air, B_home]
                        S_Wipe = [S_hover_pos, low_pos_nt]

                        ###############################################################
                        while True:
                            ik = input("pos value index = ")
                            if ik != "":
                                break

                        for i in range(len(high_pos_nt.keys())):
                            # B_Wipe.append(B_lowpos[i])
                            if i == int(ik):

                                # white self small board
                                S_Wipe.append(low_pos_nt)

                                S_Wipe.append(high_pos_nt[i])

                                S_Wipe.append(low_pos_nt)

                        Wipe_small = [S_Wipe]

                        i = 1

                        print(S_Wipe)
                        while True:
                            sin_step = input("sin step= ")
                            if sin_step != "":
                                break
                        sin_step = float(sin_step)
                        for ii, _pos in enumerate(Wipe_small):  # [[242300,17500,0]]:

                            pro_current_values = DymControl.getj("pro")

                            initial_pro_angles = DymControl.value2angle(
                                pro_current_values, "pro"
                            )
                            print(
                                "present_pro_angles",
                                initial_pro_angles,
                                pro_current_values,
                            )
                            run = 1

                            Wipe_C_one = [pro_current_values] + _pos
                            print(ii, Wipe_C_one)
                            for w in range(len(Wipe_C_one) - 1):
                                print(Wipe_C_one[w], Wipe_C_one[w + 1])
                                present_position = DymControl.sin_movej(
                                    Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, servo
                                )
                                print(
                                    "Enter",
                                    w,
                                    len(Wipe_C_one),
                                    (w + 1) / (len(Wipe_C_one) - 1),
                                )
                                if w % len(Wipe_C_one) == 0:
                                    print("Change bowl")
                                if getch() == chr(0x1B):
                                    print("Next")
                                    for servo in DymControl.servos:
                                        DymControl.Disable(servo)
                                    quit()
                            ik = ""
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

            while True:
                samples = input("samples= ")
                if samples != "":
                    break
            while True:
                trails = input("trails= ")
                if trails != "":
                    break
            while True:
                cycles = input("cycles= ")
                if cycles != "":
                    break

            trails = int(trails)
            cycles = int(cycles)
            samples = int(samples)
            recordData = RecordData(samples * cycles, 8, DymControl)  #

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
            Clow_pos_nt = [236860, 27049, -37997]
            Choverpose_nt = [244482, 15348, -37998]
            CWipe_trj_nt = [Choverpose_nt, Clow_pos_nt]
            #############################
            # new incline

            B_hover_pos = [227311, -2708, 241532]
            B_mid_air = [205796, -12754, 241532]
            B_home = [180049, 23348, 6626]
            B_lowpos_nt = [
                213360,
                90492,
                247519,
            ]  # [219166, 80523, 241503]#[214843, 75673, 241474]
            # B_lowpos_u[9]
            home_pos = [B_home, B_mid_air, B_hover_pos, B_lowpos_nt]
            return_pos = [B_hover_pos, B_mid_air, B_home]

            B_Wipe = [B_hover_pos, B_lowpos_nt]
            # B_Wipe=home_pos
            input_command = DymControl.Initialize(B_Wipe, 0.008)

            #########################################################
            Slow_pos_nt = [
                169615,
                52854,
                400406,
            ]
            S_hover_pos = [169615, 16917, 400406]
            S_home_pos = [B_home, B_mid_air, S_hover_pos]
            S_return_pos = [S_hover_pos, B_mid_air, B_home]
            S_Wipe = [S_hover_pos, Slow_pos_nt]  # S_home_pos #
            print("Reached Initial Position")
            print("Enter for automation")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()

            ########################################################
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
            Chigh_pos_nt = {}
            # Chigh_pos_nt[9]=[183680,51405,-17985]
            # Chigh_pos_nt[8]=[181768,41433,-14061]
            # Chigh_pos_nt[7]=[181108,34407,-18710]
            # Chigh_pos_nt[6]=[183680,51405,-17985]
            Chigh_pos_nt[5] = [202836, 54609, -27510]  # 9
            Chigh_pos_nt[4] = [205095, 53190, -29945]  # 8
            Chigh_pos_nt[3] = [210826, 51405, -32643]  # 7
            Chigh_pos_nt[2] = [202161, 52733, -33151]  # 3
            Chigh_pos_nt[1] = [222480, 66818, -33151]  # 2
            Chigh_pos_nt[0] = [209929, 52853, -33940]  # 0
            Cmid_pos_nt = {}
            # Cmid_pos_nt[9]=[183680,51405,-17985]
            # Cmid_pos_nt[8]=[181768,41433,-14061]
            # Cmid_pos_nt[7]=[181108,34407,-18710]
            # Cmid_pos_nt[6]=[183680,51405,-17985]
            Cmid_pos_nt[5] = [226681, 36443, -34491]  # 9
            Cmid_pos_nt[4] = [224347, 30199, -36671]  # 8
            Cmid_pos_nt[3] = [221969, 36760, -35087]  # 7
            Cmid_pos_nt[2] = [222587, 34200, -36843]  # 3
            Cmid_pos_nt[1] = [187656, 35281, -33151]  # 2
            Cmid_pos_nt[0] = [222671, 29998, -37998]  # 0
            Choverpose_nt = [244482, 15348, -37998]
            Clow_pos_nt = [236860, 27049, -37997]  # [244482,36760,-37998]
            Clow_pos_nt1 = [
                235038,
                31807,
                -40136,
            ]  # [223055,23788,-17985]#[213849,43113,-6142]
            CWipe_trj_nt = []
            i = 1
            CWipe_trj_nt = [Chigh_pos_nt[i], Cmid_pos_nt[i], Clow_pos_nt]
            ############################################
            ############################################
            # Wipe_big_latest 12/20
            B_highpos = {}
            B_lowpos = {}
            B_lowpos_u = {}
            B_lowpos_l = {}
            B_midpos = {}
            B_midpos_u = {}
            B_midpos_l = {}
            # 20
            B_lowpos_u[9] = [213360, 90492, 247519]
            B_midpos_u[9] = [147858, 128083, 247519]
            B_highpos[9] = [0, 212174, 253045]
            B_midpos_l[9] = [120507, 126015, 254592]
            B_lowpos_l[9] = [217771, 93144, 247331]
            # 25
            B_lowpos_u[8] = [205455, 94369, 247366]
            B_midpos_u[8] = [146622, 111502, 247366]
            B_highpos[8] = [3000, 200758, 254625]
            B_midpos_l[8] = [151030, 118198, 244369]
            B_lowpos_l[8] = [211100, 89935, 244369]
            # 30
            B_lowpos_u[7] = [206261, 84643, 247341]
            B_midpos_u[7] = [153730, 106682, 247341]
            B_highpos[7] = [6000, 161119, 257265]
            B_midpos_l[7] = [153730, 106682, 247341]
            B_lowpos_l[7] = [200795, 95073, 247341]
            # 35
            B_lowpos_u[6] = [206597, 87247, 242295]
            B_midpos_u[6] = [132242, 103805, 250222]
            B_highpos[6] = [9928, 172387, 253180]
            B_midpos_l[6] = [143049, 108290, 253180]
            B_lowpos_l[6] = [201387, 84319, 247298]
            # 42 #39
            B_lowpos_u[5] = [206200, 85113, 247298]
            B_midpos_u[5] = [143501, 91180, 251688]
            B_highpos[5] = [34170, 132517, 250242]
            B_midpos_l[5] = [134205, 92303, 250242]
            B_lowpos_l[5] = [197933, 85241, 242270]
            # 47   #45
            B_lowpos_u[4] = [208484, 82117, 243249]
            B_midpos_u[4] = [146497, 89136, 247331]
            B_highpos[4] = [47475, 122678, 250235]
            B_midpos_l[4] = [134205, 90303, 250242]
            B_lowpos_l[4] = [214843, 84023, 243225]  # [217933, 85241, 242270]
            # 50
            B_lowpos_u[3] = [214843, 86023, 243225]
            B_midpos_u[3] = [151503, 84057, 243225]
            B_highpos[3] = [43475, 122678, 250235]
            B_midpos_l[3] = [156425, 82057, 243225]
            B_lowpos_l[3] = [214843, 85023, 243225]
            # 55
            B_lowpos_l[2] = [210407, 86463, 243191]
            B_midpos_l[2] = [151030, 87746, 250222]
            B_highpos[2] = [67983, 102667, 250222]
            B_midpos_u[2] = [156009, 77535, 241781]
            B_lowpos_u[2] = [203849, 84457, 241781]
            # 60
            B_lowpos_u[1] = [213917, 76202, 241533]
            B_midpos_u[1] = [182622, 72688, 241503]  # [158795,68884,241533]
            B_highpos[1] = [95073, 86990, 250248]
            B_midpos_l[1] = [159871, 75008, 246264]
            B_lowpos_l[1] = [218166, 88523, 246264]
            # 65
            B_highpos[0] = [124120, 61935, 246968]  # [92057,46973,251730]#
            B_midpos_u[0] = [188622, 62688, 241503]  # [167016,39314,241481]#
            B_midpos_l[0] = [178622, 75688, 241503]  # [167016,39314,241481]#
            B_lowpos_u[0] = [
                219166,
                77523,
                241503,
            ]  # [233909,55836,241532]#[227592,59564,241532]
            B_lowpos_l[0] = [219166, 80523, 241503]
            BWipe_trj_nt = []

            for j in range(len(B_highpos.keys())):
                B_Wipe = []
                i = j
                # B_Wipe.append(B_lowpos[i])
                # if i >2 and i < 7:
                if i == 3 or i == 6:

                    B_Wipe.append(B_lowpos_u[9])
                    # B_Wipe.append(B_midpos_u[i])
                    B_Wipe.append(B_highpos[9])
                    # B_Wipe.append(B_midpos_l[i])
                    B_Wipe.append(B_lowpos_u[9])
                    print(i, "B_Wipe", B_Wipe)
                    BWipe_trj_nt.append(B_Wipe)
            Wipe_C = DymControl.Wipe_Generator(
                [[C_highpos_r, C_lowpos_r], [C_highpos_l, C_lowpos_l]], 10
            )
            print(BWipe_trj_nt)
            print("check the angles above,Enter to cont.")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()
            for ii, _pos in enumerate(BWipe_trj_nt):  # [[242300,17500,0]]:
                print("_pos", _pos)

                comments = 1
                seq_num, csv_path = DymControl.Automation_data_C(
                    int(seq_num),
                    recordData,
                    samples,
                    cycles,
                    trails,
                    _pos,
                    B_hover_pos=B_hover_pos,
                )
                # print(ii,"angle")
                print("Completed angle", ii)
                print("Enter for next")
                if getch() == chr(0x1B):
                    for servo in DymControl.servos:
                        DymControl.Disable(servo)
                        if recordData != "norecord":
                            print("Automation interrupt save")
                            recordData.save_data(csv_path, comments, trail=1)
                    break
            print("Automation_Complete")

        # Start Data Collection
        elif input_command == "r":
            i = 0
            F_present_values = {}
            x1, y1 = 2373, 2165 
            x2, y2 = 2700, 1611
            line_eq1 = DymControl.generate_line_equation(x1, y1, x2, y2)
            # DymControl.Enable("xm430F1")
            # DymControl.Disable("xm430L1")
            while 1:
                servo_type = input(
                    """Input servo type = [F1,F2,L1,L2]
                    servo_type : """
                )
                if servo_type != "":
                    break
            while i < 500:
                for servo in DymControl.servo_com[servo_type]:
                    values = DymControl.getj(servo)
                    # print("values",values)
                    F_present_values[servo] = DymControl.signed(values,servo)
                    # xm430F1_target_values = [int(line_eq1(F_present_values[2]))] #[int(-0.983* xm430L1_present_values[2] + 5001.863)]          #l[7]   line Mapping
                    # DymControl.goal["xm430F1"] = xm430F1_target_values
                    # DymControl.movej(DymControl.goal,"xm430F1")
                # print(xm430F1_target_values,F_present_values[2] , i)
                print(F_present_values , i)

                i += 1
            # low_pos1 = [205063, 24928, 2566]  # [213849,43113,-6142]#
            # high_pos1 = [149437, 39938, 2587]
            # hover_pos = [204995, 13147, 2566]
            # Wipe_trj = [hover_pos, low_pos1]
            # #####
            # B_highpos45 = [118892, 96144, 0]
            # B_lowpos45 = [205063, 24928, 0]
            # hover_pos = [204995, 13147, 2566]
            # Wipe_big = [hover_pos, B_lowpos45]  # ,B_highpos45,B_lowpos45]
            # ###
            # C_highpos_r = [189310, 54382, -10947]
            # # C_highpos_m=[54382,96144,0]
            # C_midpos = [193285, 31537, -10947]

            # C_lowpos = [217022, 30294, -10947]
            # hover_posC = [204995, 13147, 2566]
            # Wipe_C = [hover_posC, C_lowpos]  # ,B_highpos45,B_lowpos45]
            # #############################
            # # new incline

            # B_hover_pos = [227311, -2708, 241532]
            # B_mid_air = [205796, -12754, 241532]
            # B_home = [180049, 23348, 6626]
            # B_lowpos_nt = [214843, 75673, 241474]
            # home_pos = [B_home, B_mid_air, B_hover_pos]
            # return_pos = [B_hover_pos, B_mid_air, B_home]

            # B_Wipe = [B_hover_pos, B_lowpos_nt]

            # input_command = DymControl.Initialize(B_Wipe)
            # print("Enter")
            # if getch() == chr(0x1B):
            #     for servo in DymControl.servos:
            #         DymControl.Disable(servo)
            #     quit()

            # DymControl.LeaderFollowerRecord(seq_num, recordData, samples)

            seq_num += 1
            input_command = ""

        elif input_command == "i":
            ### initial positions
            """
            #Intitialize the Leader to the same angle orientation as Follower. This is to avoid jerky jump of follower.
            """
            #Changed intiarization for 10Dof
            B_start_pos       = {}    
            B_start_pos["F1"] = {'pro1': [34497, 33098, -278904, 61998], 'xm540F1': [918, 986]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876]) 
            B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2400, 2048, 2048])          
            B_start_pos["F2"] = { 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359]) 
            B_start_pos["L2"] = dict(mx1062 = [1469], xm540L2 = [1430, 1475, 2048, 2048])
            B_openhand = {}
            B_openhand["F1"] = {'pro1': [246807, 249247, -277552, 56190], 'xm540F1': [155, 1582]}#dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            B_openhand["F2"]= {'pro2': [-250336, -252991, 295683, -77844], 'xm540F2': [1901, 2313]}#{ 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])    
            
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            try:
                sin_step = 0.003
                F1_present_values = {}
                F1_current_values = {}
                L1_current_values = {}
                L1_present_values = {}
                F2_present_values = {}
                L2_current_values = {}
                L2_present_values = {}
                ##################### initialize
                for servo in DymControl.servo_com["F1"]:                                                    
                    F1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, F1_present_values)
                for servo in DymControl.servo_com["L1"]:  
                    L1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, L1_present_values)
                for servo in DymControl.servo_com["F2"]:                                                    
                    F2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, F2_present_values)
                for servo in DymControl.servo_com["L2"]:  
                    L2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, L2_present_values)
                
                # for L_F in ["F1"]:
                #     for servo in DymControl.servo_com[L_F]:
                #         L1_current_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(L1_current_values)
                #     Wipe_C_one = [L1_current_values]  + [B_openhand[L_F]]#+ [B_start_pos[L_F]]
                #     Wlen       = len(Wipe_C_one) - 1
                #     print("Wipe_C_one", Wipe_C_one)
        
                #     for w in range(Wlen):
                #         print(w, Wipe_C_one)
                #         present_position1 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                #         print("Enter", w / 2)
                #         if w % 2 == 0:
                #             print("Change angle of F1")
                #         if getch() == chr(0x1B):
                #             # for servo in DymControl.servos:
                #             #     DymControl.Disa50ble(servo)
                #             quit()

                # for L_F in ["F2"]:
                #     for servo in DymControl.servo_com[L_F]:
                #         L2_current_values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                #     print(L2_current_values)
                #     Wipe_C_one = [L2_current_values]  + [B_openhand[L_F]] #+ [B_start_pos[L_F]] 
                #     Wlen       = len(Wipe_C_one) - 1

                #     for w in range(Wlen):
                #         print(w,Wipe_C_one)
                #         present_position2 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                #         print("Enter", w / 2)
                #         if w % 2 == 0:
                #             print("Change angle of F2")
                #         if getch() == chr(0x1B):
                #             # for servo in DymControl.servos:
                #             #     DymControl.Disa50ble(servo)
                #             quit()

                # # ##############################
                sin_step = 0.003
                # # ###
                # for servo in DymControl.servo_com["F1"]:                                                    
                #     F1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, F1_present_values)
                ######### comment for dynamic values
                ##manual copy
                F1_current_values      = B_openhand["F1"]#DymControl.rounderror(F1_present_values["pro1"], 245, "mx1061")
                ## follower to leader
                L1_current_values      = DymControl.rounderror(F1_present_values["pro1"], 245, "mx1061")
                xm540L1_present_value  = L1_current_values[2:] + F1_present_values["xm540F1"]             
                #xm430L1_present_values = F1_present_values["xm540F1"][1:]    #[int((465 * F_present_values["xm430F"][0] - 2980185) / -744)]
                L1_new_values = dict(mx1061 = [L1_current_values[1]], xm540L1 = xm540L1_present_value)
                print("L1", L1_current_values)
                print(F1_present_values , L1_current_values)
                print("L1_present_values", L1_present_values)
                print("L1_new_values", L1_new_values)
                present_position1 = DymControl.sin_movej(L1_present_values, L1_new_values, sin_step, "L1")
                print("L2 move press space")
                if getch() != chr(0x20):
                    exit()
                # ##############
                # #L2
                # ################
                # for servo in DymControl.servo_com["F2"]:                                                    
                #     F2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, F2_present_values)
                # ######### comment for dynamic values
                # ########
                # L2_current_values      = DymControl.rounderror(F2_present_values["pro2"], 245, "mx1062")
                # xm540L2_present_value  = L2_current_values[1:] + F2_present_values["xm540F2"]             
                # #xm430L2_present_values = F2_present_values["xm540F2"][1:] + [int(line_eq2(F2_present_values["xm430F2"][0]))]  
                # L2_new_values = dict(mx1062 = L2_current_values[:1], xm540L2 = xm540L2_present_value)
                # print(F2_present_values , L2_current_values)
                # print("L2_present_values", L2_present_values)
                # print("L2_new_values", L2_new_values)
                # present_position = DymControl.sin_movej(L2_present_values, L2_new_values, sin_step, "L2")
                # seq_num += 1
                input_command = ""
            
            except KeyboardInterrupt:
                print('Interrupted')
                # for servo in self.servos:
                #     self.Disable(servo)
                # return ""
        elif input_command == "LF6":
            B_start_pos       = {}        #changed
            # B_start_pos["F1"] = dict(pro1 = [175861, 190882, 176331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876]) 
            # B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2712, 2048], xm430L1 = [2048, 2048, 2644])          
            # B_start_pos["F2"] = dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359]) 
            # B_start_pos["L2"] = dict(mx1062 = [1469], xm540L2 = [1707, 1536, 2048], xm430L2 = [2048, 2048, 2485])

            # B_graze_pos = dict(pro=[-17312,49371,220859])#[-17996, 63421, 214471])
            # home_pos = [B_home, B_mid_air, B_hover_pos]
            # return_pos = [B_hover_pos, B_mid_air, B_home]

            # input_command = DymControl.Initialize([B_start_pos], 0.003)
            print("leader reached")
            if getch() == chr(0x1B):
                for servo in DymControl.servos:
                    DymControl.Disable(servo)
                quit()
            
            # while True:
            #     samples = input("input samples/steps = ")
            #     if samples != "":
            #         break
            samples = 5400              #int(samples)
            DymControl.LeaderFollowerRecord(seq_num, "norecord", samples)
            seq_num += 1
            input_command = ""

        elif input_command == "h":
            B_start_pos       = {}        #changed
            # B_start_pos["F1"] = dict(pro1 = [22420, 208765, 58977], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876]) 
            # # B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2400, 2048], xm430L1 = [2048, 2048, 2644])          
            # B_start_pos["F2"] = dict(pro2 = [-14433, -214883, -57620], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359]) 
            # # B_start_pos["L2"] = dict(mx1062 = [1469], xm540L2 = [1430, 1475, 2048], xm430L2 = [2048, 2048, 2485])
            ## Enable Servo Motors
            # B_start_pos["F1"] = dict(pro1 = [22420, 208765, 58977], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876]) 
            # # B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2400, 2048], xm430L1 = [2048, 2048, 2644])          
            # B_start_pos["F2"] = dict(pro2 = [-131822, -217542, -136292], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359]) 
            # B_start_pos["L2"] = dic
            ## 
            #new home for padding
            B_start_pos["F1"] = dict(pro1 = [59890, 219124, 79661], xm540F1 = [2048, 2048, 2048], xm430F1 = [2161]) 
            # B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2400, 2048], xm430L1 = [2048, 2048, 2644])          
            B_start_pos["F2"] = dict(pro2 = [-61024, -205660, -98003], xm540F2 = [2048, 1874, 2048], xm430F2 = [2683]) 
            
            for servo in DymControl.servos:
                DymControl.Enable(servo)

            try:
                sin_step = 0.002
                F1_present_values = {}
                F1_current_values = {}
                L1_current_values = {}
                L1_present_values = {}
                F2_present_values = {}
                L2_current_values = {}
                L2_present_values = {}
                # print(F_present_values)
                ###################### initialize
                for servo in DymControl.servo_com["F1"]:                                                    
                    F1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, F1_present_values)
                for servo in DymControl.servo_com["L1"]:  
                    L1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, L1_present_values)
                for servo in DymControl.servo_com["F2"]:                                                    
                    F2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, F2_present_values)
                for servo in DymControl.servo_com["L2"]:  
                    L2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, L2_present_values)
                
                for L_F in ["F1"]:
                    # if L_F == "L":
                    for servo in DymControl.servo_com[L_F]:
                        # print("L_Fgetj",L_F)
                        L1_current_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(L1_current_values)
                    # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                    Wipe_C_one = [L1_current_values] + [B_start_pos[L_F]]
                    Wlen       = len(Wipe_C_one) - 1
                    print("Wipe_C_one",Wipe_C_one)
                    # print("sin motion")
                    # if getch() == chr(0x1B):
                    #     pass
                    for w in range(Wlen):
                        print(w,Wipe_C_one)
                        # w = Wlen - w - 1
                        # print(w,w+1,Wipe_C_one[w], Wipe_C_one[w + 1])
                        present_position1 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                        print("Enter", w / 2)
                        if w % 2 == 0:
                            print("Change angle of F1")
                        # if getch() == chr(0x1B):
                        #     # for servo in DymControl.servos:
                        #     #     DymControl.Disa50ble(servo)
                        #     quit()
                for L_F in ["F2"]:
                    # if L_F == "L":
                    for servo in DymControl.servo_com[L_F]:
                        # print("L_Fgetj",L_F)
                        L2_current_values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                    print(L2_current_values)
                    # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                    Wipe_C_one = [L2_current_values]  + [B_start_pos[L_F]]#+ [Wipe_trj[0][L_F]]
                    Wlen       = len(Wipe_C_one) - 1
                    # print("Wipe_C_one",Wipe_C_one)
                    # print("sin motion")
                    # if getch() == chr(0x1B):
                    #     pass
                    for w in range(Wlen):
                        print(w,Wipe_C_one)
                        # w = Wlen - w - 1
                        # print(w,w+1,Wipe_C_one[w], Wipe_C_one[w + 1])
                        present_position2 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                        print("Enter", w / 2)
                        if w % 2 == 0:
                            print("Change angle of F2")
                        # if getch() == chr(0x1B):
                        #     # for servo in DymControl.servos:
                        #     #     DymControl.Disa50ble(servo)
                        #     quit()
            except KeyboardInterrupt:
                print('Interrupted')
            print("home reached")#, press space to disable")
            # if getch() == chr(0x1B):
            #     # for servo in DymControl.servo_com["F1"] + DymControl.servo_com["F2"] :
            #     #     DymControl.Disable(servo)
            #     quit()
            for servo in DymControl.servo_com["F1"] + DymControl.servo_com["F2"] + DymControl.servo_com["L1"]+ DymControl.servo_com["L2"] :
                    DymControl.Disable(servo)
            exit()
            input_command = ""
        elif input_command == "e":
            for servo in DymControl.servos:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()
        elif input_command == "e1":
            for servo in DymControl.servo_com["F1"] + DymControl.servo_com["L1"]:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()
        elif input_command == "e2":
            for servo in DymControl.servo_com["F2"] + DymControl.servo_com["L2"]:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()

except KeyboardInterrupt:
    print("Interrupted")


finally:
    print("Finish Data Collection.")
