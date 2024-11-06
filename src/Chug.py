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
def get_key():
    if msvcrt.kbhit():  # Check if a key is pressed
        return msvcrt.getch().decode()  # Get the pressed key
    return None

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
               hug: hug 
                gh: get hug

            Input : """
        )  #
        # input(
        #     'First move to goal position then Press enter to start, "i" to initialize, "r" to read, "e" to finish. '
        # )
        # Change Initial Position
        #{'pro1': [34497, 33098, 4294688392, 61998], 'xm540F1': [918, 986], 'pro2': [4294925373, 4294927797, 278304, 4294889573], 'xm540F2': [1017, 3081]}
        #{'pro1': [246807, 249247, 4294689744, 56190], 'xm540F1': [155, 1582], 'pro2': [4294716960, 4294714305, 295683, 4294889452], 'xm540F2': [1901, 2313]}
        #e
        #####
        if input_command == "gh":
            values={}
            while True:
                key = get_key()
                if key:
                    print("key",key)
                if key == 'e':
                    for servo in DymControl.servos:
                        DymControl.Enable(servo)
                    print("Torque enabled.")
                elif key == 'w':
                    for servo in DymControl.servos:
                        DymControl.Disable(servo)
                    print("Torque disabled.")
                elif key == 'r':
                    # row_name = input("Enter a row name for recording the angles: ")
                    # DymControl.record_angles_to_csv(row_name)
                    print("Angles recorded.")#, row_name)
                    for servo in DymControl.servos:
                        values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                    # values =DymControl.getj("pro1")
                    print(values)
                elif key == 's':
                    print("Sin movej ex test code.")
                    for servo in DymControl.servos:
                        values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                    print(values)
                    new_value = input("Enter new value: ")
                    sin_step = 0.03
                    DymControl.sin_movej_ex(values,dict(xm540L1= [int(new_value)]), sin_step,"L1")
                    
                    print("Sin movej ex completed.")
                elif key == 'a':
                    print("Sin movej test code.")
                    for servo in DymControl.servos:
                        values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                    print(values)
                    new_value = input("Enter new value: ")
                    sin_step = 0.03
                    DymControl.sin_movej(values,dict(xm540L1= [int(new_value)]), sin_step,"L1")
                    
                    print("Sin movej completed.")
                elif key == 'z':
                    #return to 0
                    for servo in DymControl.servos:
                        for servo in DymControl.servos:
                            values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                            unsinged = DymControl.getj(servo)
                        print(values)
                        print("unsinged",unsinged)
                        print("moving servo",servo)
                        DymControl.Enable(servo)
                        values[servo] = [0]
                        print(values,values[servo])
                        DymControl.movej(values,servo)
                    print("moved to 0")
                    

                elif key == 'q':
                    print("Exiting program.")
                    break
        elif input_command == "hug":
            
            pass
        elif input_command == "g":
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
        elif input_command == "i":
            ### initial positions
            """
            #Intitialize the Leader to the same angle orientation as Follower. This is to avoid jerky jump of follower.
            """
            B_start_pos       = {}    
            B_start_pos["F1"] = {'pro1': [34497, 33098, -278904, 61998], 'xm540F1': [918, 986]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876]) 
            B_start_pos["L1"] = dict(mx1061 = [2692], xm540L1 = [2810, 2400, 2048], xm430L1 = [2048, 2048, 2644])          
            B_start_pos["F2"] = { 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359]) 
            B_start_pos["L2"] = dict(mx1062 = [1469], xm540L2 = [1430, 1475, 2048], xm430L2 = [2048, 2048, 2485])
            B_openhand = {}
            B_openhand["F1"] = {'pro1': [246807, 249247, -277552, 56190], 'xm540F1': [155, 1582]}#dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            B_openhand["F2"]= {'pro2': [-250336, -252991, 295683, -77844], 'xm540F2': [1901, 2313]}#{ 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])    
            
            B_holdhand = {}
            B_holdhand["F1"] = {'pro1': [225430, 227108, -285742, 132112], 'xm540F1': [243, 1308]}
            #{'pro1': [246807, 249247, -277552, 56190], 'xm540F1': [155, 1582]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            #{'pro1': [34497, 33098, -278904, 61998], 'xm540F1': [918, 986]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            B_holdhand["F2"]= { 'pro2': [-233858, -237273, 311432, -134410], 'xm540F2': [2055, 2892]}#{ 'pro2': [-250336, -252991, 295683, -77844], 'xm540F2': [1901, 2313]}#{ 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])
            #{ 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])

            # B_press = {}
            # press_val = 6000
            # B_press["F1"] = {'pro1': [225430, 227108, -285742, 132112], 'xm540F1': [243, 1308]}
            # #{'pro1': [246807, 249247, -277552, 56190], 'xm540F1': [155, 1582]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            # #{'pro1': [34497, 33098, -278904, 61998], 'xm540F1': [918, 986]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            # B_press["F2"]= { 'pro2': [-233858, -237273, 311432, -134410], 'xm540F2': [2055, 2892]}#{ 'pro2': [-250336, -252991, 295683, -77844], 'xm540F2': [1901, 2313]}#{ 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])
            
            # B_lift = {}
            # B_lift["F1"] = {'pro1': [34497, 33098, -278904, 61998], 'xm540F1': [918, 986]}##dict(pro1 = [200861, 190882, 196331], xm540F1 = [2048, 2048, 2048], xm430F1 = [1876])
            # B_lift["F2"]= { 'pro2': [-41923, -39499, 278304, -77723], 'xm540F2': [1017, 3081]} #dict(pro2 = [-205156, -221447, -201723], xm540F2 = [2048, 2048, 2048], xm430F2 = [2359])

            for servo in DymControl.servos:
                DymControl.Enable(servo)

            try:
                sin_step = 0.005
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
                # for servo in DymControl.servo_com["L1"]:  
                #     L1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, L1_present_values)
                for servo in DymControl.servo_com["F2"]:                                                    
                    F2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(servo, F2_present_values)
                # for servo in DymControl.servo_com["L2"]:  
                #     L2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, L2_present_values)
                
                for L_F in ["F1"]:
                    for servo in DymControl.servo_com[L_F]:
                        F1_current_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                    print(F1_current_values)
                    Wipe_C_one = [F1_current_values] + [B_start_pos[L_F]] + [B_openhand[L_F]] + [B_holdhand[L_F]] #+ # + [B_openhand[L_F]] + [B_start_pos[L_F]]
                    Wlen       = len(Wipe_C_one) - 1
                    print("Wipe_C_one", Wipe_C_one)
        
                    for w in range(Wlen):
                        print(w, Wipe_C_one)
                        present_position1 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                        print("Enter", w / 2)
                        if w % 2 == 0:
                            print("Change angle of F1")
                        if getch() == chr(0x1B):
                            # for servo in DymControl.servos:
                            #     DymControl.Disa50ble(servo)
                            quit()

                for L_F in ["F2"]:
                    for servo in DymControl.servo_com[L_F]:
                        L2_current_values[servo] = DymControl.signed(DymControl.getj(servo),servo)
                    print(L2_current_values)
                    Wipe_C_one = [L2_current_values]  + [B_start_pos[L_F]] + [B_openhand[L_F]] + [B_holdhand[L_F]] # + [B_openhand[L_F]] + [B_start_pos[L_F]]
                    Wlen       = len(Wipe_C_one) - 1

                    for w in range(Wlen):
                        print(w,Wipe_C_one)
                        present_position2 = DymControl.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                        print("Enter", w / 2)
                        if w % 2 == 0:
                            print("Change angle of F2")
                        if getch() == chr(0x1B):
                            # for servo in DymControl.servos:
                            #     DymControl.Disa50ble(servo)
                            quit()

                # # # ##############################
                # x1, y1 = 2373, 2165 
                # x2, y2 = 2728, 1611
                # line_eq1 = DymControl.generate_line_equation_y(x1, y1, x2, y2)
                # x3, y3 = 2410, 2685
                # x4, y4 = 2700, 2188 
                # line_eq2 = DymControl.generate_line_equation_y(x3, y3, x4, y4)
                # # ###
                # for servo in DymControl.servo_com["F1"]:                                                    
                #     F1_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, F1_present_values)
                # ######### comment for dynamic values
                # ## follower to leader
                # L1_current_values      = DymControl.rounderror(F1_present_values["pro1"], 245, "mx1061")
                # xm540L1_present_value  = L1_current_values[1:] + F1_present_values["xm540F1"][:1]              
                # xm430L1_present_values = F1_present_values["xm540F1"][1:] + [int(line_eq1(F1_present_values["xm430F1"][0]))]    #[int((465 * F_present_values["xm430F"][0] - 2980185) / -744)]
                # L1_new_values = dict(mx1061 = L1_current_values[:1], xm540L1 = xm540L1_present_value , xm430L1 = xm430L1_present_values)
                # print(F1_present_values , L1_current_values)
                # print("L1_present_values", L1_present_values)
                # print("L1_new_values", L1_new_values)
                # present_position1 = DymControl.sin_movej(L1_present_values, L1_new_values, sin_step, "L1")
                # print("L2 move press space")
                # if getch() != chr(0x20):
                #     exit()
                # ##############
                # #L2
                # ################
                # for servo in DymControl.servo_com["F2"]:                                                    
                #     F2_present_values[servo] = DymControl.signed(DymControl.getj(servo), servo)
                #     print(servo, F2_present_values)
                # ######### comment for dynamic values
                # ########
                # L2_current_values      = DymControl.rounderror(F2_present_values["pro2"], 245, "mx1062")
                # xm540L2_present_value  = L2_current_values[1:] + F2_present_values["xm540F2"][:1]              
                # xm430L2_present_values = F2_present_values["xm540F2"][1:] + [int(line_eq2(F2_present_values["xm430F2"][0]))]  
                # L2_new_values = dict(mx1062 = L2_current_values[:1], xm540L2 = xm540L2_present_value , xm430L2 = xm430L2_present_values)
                # print(F2_present_values , L2_current_values)
                # print("L2_present_values", L2_present_values)
                # print("L2_new_values", L2_new_values)
                # present_position = DymControl.sin_movej(L2_present_values, L2_new_values, sin_step, "L2")
                # seq_num += 1
                # input_command = ""
            
            except KeyboardInterrupt:
                print('Interrupted')
                # for servo in self.servos:
                #     self.Disable(servo)
                # return ""
        elif input_command == "e":
            for servo in DymControl.servos:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()

            for servo in DymControl.servo_com["F2"] + DymControl.servo_com["L2"]:
                DymControl.Disable(servo)
            DymControl.ClosePort()
            input_command = ""
            exit()
except KeyboardInterrupt:
    print("Interrupted")


finally:
    print("Finish Data Collection.")
