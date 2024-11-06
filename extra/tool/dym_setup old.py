# from macpath import join
#
# "a" variable means angle
# "v" variable means value of servo
#

import datetime
import os
import time

import numpy as np
from dynamixel_sdk import PacketHandler, PortHandler

COMM_SUCCESS = 0
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


class Control:
    def __init__(self) -> None:
        # Dynamixel Settings
        self.DEVICENAME = dict(L="COM5", F="COM4")  # list all comports
        self.servo_com = dict(
            L=["xm540"], F=["xm430","mx106"]
            
        )  # define leader follower, list all servo types
        self.DXL_IDs = dict(xm540=[1,2,3],xm430=[3,4,5,6,7],mx106=[1,2])
        self.PROTOCOL_VERSION = 2.0
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 5
        self.portHandler = dict(
            L=PortHandler(self.DEVICENAME["L"]), F=PortHandler(self.DEVICENAME["F"])
        )
        self.a1, self.a2, self.b1, self.b2 = [], [], [], []
        self.stat = dict(xm540={}, pro={}, mx106={},xm430={})
        self.run = [True, True, True]  # only once for while loop
        self.times = {}
        self.timediff = {}
        # change !!
        self.d = dict()
        angle_list = range(-180, 180)
        theta_ranges = [round(n * 11.375) for n in range(-180, 180)]
        self.d["xm540"] = dict(zip(theta_ranges, angle_list))

        angle_list1 = range(-180, 180)
        theta_ranges1 = [round(n * 2788.461111111) for n in range(-180, 180)]
        self.d["pro"] = dict(zip(theta_ranges1, angle_list1))

        angle_list = range(-180, 180)
        theta_ranges = [round(n * 11.375) for n in range(-180, 180)]
        self.d["xm430"] = dict(zip(theta_ranges, angle_list))

        angle_list = range(-180, 180)
        theta_ranges = [round(n * 11.375) for n in range(-180, 180)]
        self.d["mx106"] = dict(zip(theta_ranges, angle_list))

        # angle_list2 = range(-501923,501923)
        # theta_ranges2 = [n*0.00035862 for n in range(-501923,501923)]

        # self.d["pro"] = dict(zip(angle_list2, theta_ranges2))

        # angle_list1 = range(-2048,2048)
        # theta_ranges1 = [n*0.08789 for n in range(-2048,2048)]

        # self.d["xm540"] = dict(zip(angle_list1, theta_ranges1))

        """
        For data collextion
        x1=xm540_present_values
        x2=xm540_filtered_values
        y1=pro_present_values
        y2=pro_filtered_values
        a1,b1 = before fitering for xm540 and pro
        a2,b2
        stat to identify direction
        """
        # Dynamixel Pro,xm540 Setting -> Base Joint, Shoulder Joint, Elbow Joint
        self.ADDR_TORQUE_ENABLE = dict(pro=512, xm540=64,mx106=64,xm430=64)
        self.ADDR_GOAL_POSITION = dict(pro=564, xm540=116,mx106=116,xm430=116)
        self.ADDR_PRESENT_POSITION = dict(pro=580, xm540=132,mx106=132,xm430=132)
        self.DXL_MINIMUM_POSITION_VALUE = dict(pro=-501923, xm540=0,mx106=0,xm430=0)
        self.DXL_MAXIMUM_POSITION_VALUE = dict(pro=501923, xm540=4095,mx106=4095,xm430=4095)
        self.ADDR_Present_Current = dict(pro=574, xm540=126,mx106=126,xm430=126)
        # self.ADDR_Current_Limit = dict(pro=38, xm540=4095)
        self.BAUDRATE = 115200  # 1000000#  # 57600
        ##
        # check devices ports connectivity
        self.bias = dict(xm540=2048, pro=0, mx106=2048, xm430=2048)
        self.goal = dict(xm540=[2920, 2223, 2022], pro=[132227, 48997, 500],mx106=[2048,2048],
        xm430=[2048,2920, 2223, 2022])
        self.servos = []
        for device in list(self.DEVICENAME.values()):
            print(device)
            servo = self.servo_com[                
                    list(self.DEVICENAME.keys())[
                        list(self.DEVICENAME.values()).index(device)
                    ]
            ]
            print(servo)
            if self.portIsUsable(device):
                print("Isport",servo)
                self.servos = self.servos + servo

        print("servos available:", self.servos)

        # Open port
        for servo in self.servos:
            if self.portHandler[self.servos_LF(servo)].openPort():
                print("Succeeded to open the port " + servo)
            else:
                print("Failed to open the port " + servo)
                # print("Press any key to terminate...")
                # getch()
                quit()

            ##imp : Utilizing default baudrate set from dynamixel wizard instead of setting it everytime.
            # Set port baudrate
            if self.portHandler[self.servos_LF(servo)].setBaudRate(self.BAUDRATE):
                print("Succeeded to change the baudrate " + servo)
            else:
                print("Failed to change the baudrate " + servo)
                # print("Press any key to terminate...")
                # getch()
                quit()

        ###############################
        # {Servo : [id]} = list dict variables
        ###############################
        # self.DXL_IDs[servo].append(i + 1)
        
        
        self.packetHandlers = {}
        self.reaches = {}

        self.collection = {}
        # self.present_angles = {}
        for servo in self.servos:
            self.packetHandlers[servo] = []
            
            self.collection[servo] = {}
            self.reaches[servo] = []

            # self.present_angles[servo]=[]
            for i in range(len(self.DXL_IDs[servo])):
                

                self.packetHandlers[servo].append(PacketHandler(self.PROTOCOL_VERSION))
                self.collection[servo][i + 1] = []
                self.reaches[servo].append(1)
                # self.present_angles[servo].append()
        # print("self.DXL_IDs",self.DXL_IDs,"self.packetHandlers",self.packetHandlers,"self.reaches",self.reaches)
    def servos_LF(self,servo):
        LF = list(self.servo_com.values())
        for i,ser_ls in enumerate (LF):
            if servo in ser_ls:
                return list(self.servo_com.keys())[i]

    def portIsUsable(self, portName):
        import serial
        from serial import SerialException
        # print(portName)
        try:
            ser = serial.Serial(port=portName)
            print(portName,True)
            return True

        except:
            print(False)
            return False


    def getj(self, servo):  #
        self.present_angles = []
        for i in range(len(self.DXL_IDs[servo])):
            (
                self.dxl_present_position,
                self.dxl_comm_result,
                self.dxl_error,
            ) = self.packetHandlers[servo][i].read4ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_GOAL_POSITION[servo],
            )
            self.present_angles.append(self.dxl_present_position)

            ##debug for read error

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getTxRxResult(self.dxl_comm_result),i
                )
                print("read error: try again")
                if getch() == chr(0x1b):
                    for servo in self.servos:
                        self.Disable(servo)
                    quit()
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getRxPacketError(self.dxl_error),i
                )

        print("self.present_angles-getj",self.present_angles)
        return self.present_angles

    def getCurrent(self, servo):
        self.current = []
        for i in range(len(self.DXL_IDs[servo])):
            (
                self.dxl_present_current,
                self.dxl_comm_result,
                self.dxl_error,
            ) = self.packetHandlers[servo][i].read2ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_Present_Current[servo],
            )
            if self.dxl_present_current > 0x7FFF:
                self.dxl_present_current = self.dxl_present_current - 65536

            # self.current.append(self.dxl_present_current)
            # print(self.current)
            ##debug for read error

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getTxRxResult(self.dxl_comm_result)
                )
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getRxPacketError(self.dxl_error)
                )

        # print(joint_angles)
        return self.current

    def movej(self, goal, servo) -> None:
        for i in range(len(self.DXL_IDs[servo])):
            print(servo,self.DXL_IDs[servo][i],goal[servo][i])
            self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo][
                i
            ].write4ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_GOAL_POSITION[servo],
                int(goal[servo][i]),
            )
            ##debug for read error

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getTxRxResult(self.dxl_comm_result)
                )
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo][i].getRxPacketError(self.dxl_error)
                )

    def step_movej(self, goal, servo, step) -> None:

        if servo == "pro":
            present = self.getj(servo)

            for i in range(len(self.DXL_IDs[servo])):
                goal_pro = self.toSigned32(
                    self.angle_diff(goal, present[i], i, servo), servo
                )
                # print(self.stat[servo][self.DXL_IDs[servo][i]])
                if self.stat[servo][self.DXL_IDs[servo][i]]:
                    goal_pro = goal_pro + step
                else:
                    goal_pro = goal_pro - step
                print("targetpro", goal[servo][i], "presentpro", goal_pro)
                self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo][
                    i
                ].write4ByteTxRx(
                    self.portHandler[self.servos_LF(servo)],
                    self.DXL_IDs[servo][i],
                    self.ADDR_GOAL_POSITION[servo],
                    goal_pro,
                )
                # self.reach(target,servo)

    def sin_movej(self, initial_values, Final_values, sin_step, servo):
        ang_values = []
        initial_values = self.signed(initial_values, servo)
        Final_values = self.signed(Final_values, servo)
        angle_displacement = np.array(initial_values) - np.array(Final_values)
        print("angle_displacement", angle_displacement)
        step_size = [0, 0, 0]
        # sin_step=0.01
        for tta in np.arange(0, np.pi, sin_step):
            run = 1
            step_size = step_size + (angle_displacement / 2) * np.sin(tta) * sin_step
            ang_values = initial_values - step_size
            self.goal[servo] = ang_values
            self.movej(self.goal, servo)
            while run:
                run = self.reach(self.goal, servo)
                # run=0
            print("pro_angles", ang_values, self.value2angle(ang_values, servo))
        print("initial_pro_angles", self.value2angle(initial_values,servo))
        print("Final_angle_values", self.value2angle(Final_values, servo))

        return ang_values

    def angle_diff(self, goal, present, i, servo):

        if servo == "pro":
            present_ = self.toSigned32(present, servo)
            # print("present",present_,goal[servo],present_ >= goal[servo][i],i)
            if present_ >= goal[servo][i]:
                self.stat[servo][self.DXL_IDs[servo][i]] = 0
            else:
                self.stat[servo][self.DXL_IDs[servo][i]] = 1
            # elif servo=="xm540":
            #     self.stat[servo][self.DXL_IDs[servo][i]]=1
        return present_  # self.stat[servo][self.DXL_IDs[servo]]


    def sin_record_wipe(self, initial_values, Final_values, samples,cycles,recordData,image_path,image_path_side, servo):
        ang_values = []
        initial_values = self.signed(initial_values, servo)
        for cyc in range(cycles):
            print("cycle",cyc)
            for final_value in Final_values:
                
                angle_displacement = np.array(initial_values) - np.array(final_value)
                print("initial_values,final_value",initial_values,final_value)
                print("angle_displacement", angle_displacement)
                
                step_size = [0, 0, 0]
                sin_step = np.pi/(samples/len(Final_values))
                samp_len = np.arange(0, np.pi, sin_step)
                print("len(samp_len)",len(samp_len),"sin_step",sin_step)
                i=0
                # sin_step=0.01
                
                while i < len(samp_len):
                    run = 1
                    step_size = step_size + (angle_displacement / 2) * np.sin(samp_len[i]) * sin_step
                    ang_values = initial_values - step_size
                    self.goal[servo] = ang_values
                    self.movej(self.goal, servo)

                    if recordData != "norecord":
                        recordData.record(image_path, image_path_side, "pro")
                        # self.get_time(time.time(), i, "record_data")
                    while run:                
                        run = self.reach(self.goal, servo)            
                    # print("pro_angles", ang_values, self.value2angle(ang_values, "pro"))
                    i+=1
                initial_values = final_value
                print("initial_pro_angles", self.value2angle(initial_values, "pro"))
                # print("Final_angle_values", self.value2angle(Final_values, "pro"))

        return ang_values

    def Wipe_Generator(self,Wipe_Angles,step):
        #mainly for generating wipe positions for high postion 
        Wipe_C=[]
        for x1,y1 in zip(Wipe_Angles[0],Wipe_Angles[1]):
            print(x1,y1)
            # A=dict(A1=[],A2=[],A3=[])
            A=[[],[],[]]
            
            for i,q in enumerate(A):
                diff = x1[i]-y1[i]
                # print(diff,i,q)

                if diff >  0:
                    # print("range1",list(range (0,diff+round(diff/step),round(diff/step))))
                    for j,x in enumerate(range (0,diff+round(diff/step),round(diff/step))):
                        # print(j,x)
                        A[i].append(y1[i] + x)
                    
                        # print(len(A[i]))
                else:
                    # print("range2",list(range (diff,round(abs(diff)/step),round(abs(diff)/step))))
                    for j,x in enumerate(range (diff,round(abs(diff)/step),round(abs(diff)/step))):
                        # print(j,x)
                        A[i].append(x1[i] - x)
                    
                        # print(len(A[i]))
            # print(A)
            Wipe_C.append(list(zip(A[0],A[1],A[2])))
        return list(zip(*Wipe_C))


    def Enable(self, servo):
        for i in range(len(self.DXL_IDs[servo])):
            print(servo,i,self.DXL_IDs[servo])
            self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo][
                i
            ].write1ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_TORQUE_ENABLE[servo],
                self.TORQUE_ENABLE,
            )
            print("Enable " + servo,self.DXL_IDs[servo][i],self.ADDR_TORQUE_ENABLE[servo])
            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s" % self.packetHandlers[servo][i].getTxRxResult(self.dxl_comm_result),i
                )
            elif self.dxl_error != 0:
                print("%s" % self.packetHandlers[servo][i].getRxPacketError(self.dxl_error),i)

    def Disable(self, servo):
        for i in range(len(self.DXL_IDs[servo])):
            self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo][
                i
            ].write1ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_TORQUE_ENABLE[servo],
                self.TORQUE_DISABLE,
            )
        print("Disable " + servo)
        if self.dxl_comm_result != COMM_SUCCESS:
            print(
                "%s" % self.packetHandlers[servo][i].getTxRxResult(self.dxl_comm_result)
            )
        elif self.dxl_error != 0:
            print("%s" % self.packetHandlers[servo][i].getRxPacketError(self.dxl_error))

    def value2angle(self, dxl, servo):
        """
        # mapping from servo position value to angle
        """
        d = self.d[servo]
        prepos = []
        for ang in dxl:
            ang = self.toSigned32(ang, servo) - self.bias[servo]
            # print(ang)
            prepos.append(
                list(d.values())[
                    list(d.keys()).index(min(d.keys(), key=lambda x: abs(x - ang)))
                ]
            )

        return prepos

    # def convertLF(self,dxl,servo):!!WIP
    #     d=self.d[servo]
    #     # dxl=self.value2angle(dxl,servo)
    #     prepos = []
    #     for ang in dxl:
    #         # ang= self.toSigned32(ang,servo)
    #         prepos.append(list(d.keys())[list(d.values()).index(min(d.values(), key=lambda x:abs(x-ang)))] + self.bias[servo])

    #     return prepos

    def reach(self, goal, servo): 
        # print("in reach")
        # self.reach[servo]={}
        # compare present and goal values until the servo is reached
        values = self.getj(servo)

        if servo == "pro":
            # break
            # pass

            # print("in")
            for i in range(len(self.DXL_IDs[servo])):
                degre = self.toSigned32(values[i], servo)
                goal[servo][i] = self.toSigned32(goal[servo][i], servo)
                # if values[i]%10 ==0:

                # print(
                #     "[PROID:%03d] GoalPos:%03d AnglePos:%03d "
                #     % (self.DXL_IDs[servo][i], goal[servo][i], degre)
                # )  # , self.value2angle(d,values[i])))degree PresentPos:%03d
                # print(self.reaches[servo], goal[servo][i] - degre)
                if not (
                    abs(goal[servo][i] - degre) > 100
                ):  # or (self.reaches[servo][0] or self.reaches[servo][1] or self.reaches[servo][2]):#self.DXL_MOVING_STATUS_THRESHOLD:
                    # print("reached" , i)
                    self.reaches[servo][i] = 0

                    # return self.reaches[servo][0] or self.reaches[servo][1] or self.reaches[servo][2]
        elif servo == "xm540":
            degre = self.value2angle(values, servo)
            print(degre, values, self.DXL_IDs[servo], goal[servo])
            for i in range(3):  # len(self.DXL_IDs[servo])
                # print(i)
                # if values[i]%10 ==0:
                # value=self.toSigned32(values[i])
                print(
                    "[xm540ID:%03d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d"
                    % (self.DXL_IDs[servo][i], goal[servo][i], values[i], degre[i])
                )
                # print("reaching" ,i, self.reaches[servo],goal[servo][i] - values[i])

                if not (
                    abs(goal[servo][i] - values[i]) > self.DXL_MOVING_STATUS_THRESHOLD
                ):
                    self.reaches[servo][i] = 0
                    # print("reached" , self.reaches[servo],goal[servo][i], values[i],goal[servo][i] - values[i])

                    # return (self.reaches[servo][0] | self.reaches[servo][1] | self.reaches[servo][2])
        return any(self.reaches[servo])
        # return 1

    def toSigned32(self, n, servo="pro"):
        # print(n)
        n = int(n) & 0xFFFFFFFF
        # if n >= 501923:
        #     print("n",n , (n | (-(n & 0x80000000))) )
        return n | (-(n & 0x80000000))

    def signed(self, actual_values, servo):
        self.sign_values = []
        for id in range(len(self.DXL_IDs[servo])):
            self.sign_values.append(self.toSigned32(actual_values[id], servo))
        return self.sign_values

    def LiveLPF_initialize(self, d=2, fs=30, Wn=2):
        ##Live low pass filter
        # import pandas as pd
        # from sklearn.metrics import mean_absolute_error as mae
        import scipy

        from utils.digitalfilter import LiveSosFilter

        self.sos = {}
        self.live_sosfilter = {}
        for servo in self.servos:
            self.sos[servo] = {}
            self.live_sosfilter[servo] = {}
            for id in range(len(self.DXL_IDs[servo])):
                # print(id)
                self.sos[servo][self.DXL_IDs[servo][id]] = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )
                # print(self.sos)
                self.live_sosfilter[servo][self.DXL_IDs[servo][id]] = LiveSosFilter(
                    self.sos[servo][self.DXL_IDs[servo][id]]
                )
                print("self.goal[servo][id]", self.goal[servo][id])
        return self.live_sosfilter

    def Filter(self, joint_angles, servo):
        self.filter_angles = []  # dict(xm540 = [],pro = [])
        for id, y in enumerate(joint_angles):
            # print(id,y)
            if servo == "pro":
                filter_value = self.live_sosfilter[servo][self.DXL_IDs[servo][id]](y)
                if self.run[id]:
                    # time.sleep(0.25)
                    while self.run[id]:

                        filter_value = self.live_sosfilter[servo][
                            self.DXL_IDs[servo][id]
                        ](y)
                        print("filter",y-filter_value)
                        if abs(y - filter_value) < 3:
                            self.run[id] = False
                    self.run[id] = False

                self.filter_angles.append(filter_value)
            elif servo == "xm540":
                self.filter_angles.append(
                    self.live_sosfilter[servo][self.DXL_IDs[servo][id]](y)
                )

        return self.filter_angles

    def valuesforplot(self, time, y1, y2, servo):
        """
        x1=xm540_present_values
        x2=xm540_filtered_values
        y1=pro_present_values
        y2=pro_filtered_values
        a1,b1 = before fitering for xm540 and pro
        a2,b2 = after filtering xm540 , pro
        """
        # print(time,y1,y2,servo)
        for i, id in enumerate(self.DXL_IDs[servo]):

            # for t,b1,b2 in zip(time,y1[i],y2[i]):
            self.collection[servo][id].append([time, y1[i], y2[i]])

    ##############
    def liveplot(self, name, seq_num):
        import matplotlib.pyplot as plt
        import numpy as np

        print("Check the plots")
        fmt = lambda x1: "{:.3f}".format(x1)
        for i in self.DXL_IDs.keys():
            if i == "pro":
                for j in self.DXL_IDs[i]:
                    time, data, y_live_sosfilt = np.array(self.collection[i][j]).T
                    plt.figure(figsize=[128, 80])
                    t = [fmt(g - time[0]) for g in time]  # np.arange(0, len(time), 1)
                    # print(j,time,t)
                    plt.plot(t, data, lw=2, label="Noisy signal")
                    plt.scatter(t, data, lw=2, label="Noisy signal")
                    # plt.plot(time, y_live_sosfilt, lw=2, label="SciPy lfilter")
                    plt.plot(
                        t,
                        y_live_sosfilt,
                        lw=3,
                        label="LiveLFilter-" + name + "-" + i + "-" + str(j),
                    )

                    plt.legend(
                        loc="lower center",
                        bbox_to_anchor=[0.5, 1],
                        ncol=2,
                        fontsize="smaller",
                    )
                    plt.scatter(
                        t,
                        y_live_sosfilt,
                        lw=3,
                        label="LiveLFilter-" + name + "-" + i + "-" + str(j),
                    )
                    plt.xticks(t, rotation=90)
                    plt.xlabel("Time / s")
                    plt.ylabel("Amplitude")
                    #

                    plt.tight_layout()
                    # plt.pause(0.25)
                    plt.savefig(
                        "./src/fig/_t_"
                        + str(seq_num)
                        + "__"
                        + name
                        + "-"
                        + i
                        + "-"
                        + str(j)
                        + ".pdf"
                    )

                    # plt.show()
                    # self.collection[i][j]=[]

    def rounderror(self, values, multiplier, servo):
        # div=m/245
        if servo == "pro":
            self.round_a = []
            for v in values:
                div = multiplier / 245.0
                val = round((v - self.bias["xm540"]) / div)
                # print("v",v,val)
                val = val * multiplier
                self.round_a.append(val)
            return self.round_a
        if servo == "xm540":
            self.round_a = []
            for v in values:
                div = multiplier / 245.0
                # print("v",v)

                val = round(self.toSigned32(v, "pro") / div)  #
                # print("self.toSigned32(v)", v, self.toSigned32(v, "pro"))
                val = val / multiplier
                self.round_a.append(val + self.bias["xm540"])
            return self.round_a

    def get_seq(self, times, name):
        # open output file for writing
        import json

        with open(
            "./src/times/Timediff_" + str(times) + "_" + str(name) + ".json", "w"
        ) as filehandle:
            json.dump(self.timediff, filehandle)
        with open(
            "./src/angles/Collection_e_" + str(times) + "_" + str(name) + ".json", "w"
        ) as filehandle:
            json.dump(self.collection, filehandle)

    def get_time(self, time, seq_num, loc):
        if seq_num in self.times.keys():
            # print(self.times[seq_num])
            self.times[seq_num].append(time)
            self.timediff[seq_num][loc] = (
                self.times[seq_num][-1] - self.times[seq_num][-2]
            )
            # print("timechange" + str (self.times[seq_num].index(time))+" : " + loc+" = ",self.times[seq_num][-1] - self.times[seq_num][-2])
            if loc == "finish":
                # print("total time " + str(seq_num), self.times[seq_num][-1] - self.times[seq_num][0])
                self.timediff[seq_num]["total_time"] = (
                    self.times[seq_num][-1] - self.times[seq_num][0]
                )
        else:
            self.times[seq_num] = [time]
            self.timediff[seq_num] = {}
        # self.times=

    def ClosePort(self):
        # Close port
        for servo in self.servos:
            self.portHandler[self.servos_LF(servo)].closePort()
            print("Port Closed for " + servo)

    #########################################################################################
    # Control Code for dynamixel
    #########################################################################################
    def Initialize(self,Wipe_trj = [
                        [132474, 48933, -149],
                        [251527, 26955, -1080]
                    ]):
        """
        #Intitialize the Leader to the same angle orientation as Follower. This is to avoid jerky jump of follower.
        """
        ## Enable Servo Motors
        for servo in self.servos:
            self.Enable(servo)

        try:
            pro_present_values = self.getj("pro")
            # print(pro_present_values)
            xm540_align_angles = self.rounderror(Wipe_trj[-1], 245.0, "xm540")
            # print(self.value2angle(xm540_align_angles,"xm540"))
            print(
                "xm540_align_angles",
                self.value2angle(xm540_align_angles, "xm540"),
                "pro_angles",
                self.value2angle(pro_present_values, "pro"),
            )
            self.goal = dict(xm540=xm540_align_angles, pro=pro_present_values)
            self.wipe_angle=Wipe_trj[-1]
            # print(goal)
            for servo in self.servos:
                if servo == "pro":
                    # Wipe_trj = [
                    #     [132474, 48933, -149],
                    #     [251527, 26955, -1080]
                    #     # [255085,26513,0]
                    #     # ,
                    #     # [164043, 62878, 17687],
                    #     # [246194, 41472, 44186],
                    #     # [140788, 67811, 50354],
                    #     # [159661, 64144, -38188],
                    #     # [250000, 26955, -1080],
                    # ]
                    for pro_ang in Wipe_trj:  # [[242300,17500,0]]:
                        pro_current_values = self.getj("pro")
                        initial_pro_angles = self.value2angle(pro_current_values, "pro")
                        print("present_pro_angles", initial_pro_angles)
                        run = 1
                        sin_step = 0.01
                        present_position = self.sin_movej(
                            pro_current_values, pro_ang, sin_step, servo
                        )

                elif servo == "xm540":
                    self.movej(self.goal, servo)
                    run = 1
                    while run:
                        run = self.reach(self.goal, servo)
                        # print(run,self.present_angles,self.value2angle(self.present_angles,"xm540"))
                        # break
        except KeyboardInterrupt:
            # print('Interrupted')
            for servo in self.servos:
                self.Disable(servo)
            return ""

    def LeaderFollowerRecord(self, seq_num, recordData, samples):
        """
        Control code as follows:
        - Read from the Leader
        - Convert Leader angle(value) to Follower angle(value)
        - Smooth the follower angles with low pass filter
        - Collect the plot values
        - Move the follower to the filtered target angles
        - record image and motor data
        """
        d = 2
        fs = 20
        Wn = 2.0
        self.Disable("xm540")
        self.Enable("pro")
        self.LiveLPF_initialize(d=d, fs=fs, Wn=Wn)
        if recordData != "norecord":
            # Create D;irectory
            csv_path, image_path, image_path_side = recordData.create_datadir(seq_num)
        self.stats = 0
        # print("Started Leader Follower Control")
        try:
            i = 0
            while i < samples:

                self.get_time(time.time(), i, "start")
                xm540_present_values = self.getj("xm540")
                self.get_time(time.time(), i, "get_xm540_angles")
                self.valuesforplot(
                    time.time(), xm540_present_values, xm540_present_values, "xm540"
                )
                self.get_time(time.time(), i, "valuesforplot_xm540_angles")
                # #####Get current test code - remove later

                # xm540_current_values=self.getj("xm540")
                # self.get_time(time.time(),i,"get_xm540_current")
                # print("xm540_current_values",xm540_current_values)
                # self.valuesforplot(time.time(),xm540_current_values,xm540_current_values,"xm540")
                # self.get_time(time.time(),i,"xm540_angles")
                # #####
                pro_target_values = self.rounderror(
                    xm540_present_values, 245, "pro"
                )  # self.convertLF(xm540_angles,"pro")
                self.get_time(time.time(), i, "get_pro_target_values")
                pro_filtered_values = self.Filter(pro_target_values, "pro")
                self.get_time(time.time(), i, "filter_pro_values")
                self.valuesforplot(
                    time.time(), pro_target_values, pro_filtered_values, "pro"
                )
                self.get_time(time.time(), i, "valuesforplot_pro")
                self.goal = dict(xm540=xm540_present_values, pro=pro_filtered_values)
                # self.movej(self.goal, "pro")
                self.get_time(time.time(), i, "Move_Follower")
                # print(xm540_present_values,pro_target_values)
                # print("xm540 ",self.value2angle(xm540_present_values,"xm540"),"PRO",self.value2angle(pro_target_values,"pro"))
                # self.get_time(time.time(),i,"value2angle")

                # print( str(i) +" ----------------------------")
                if recordData != "norecord" :
                    recordData.record(image_path, image_path_side, "pro")
                    self.get_time(time.time(), i, "record_data")
                else:
                    # print("no record stats")
                    self.stats = 1
                    # if self.stat & (recordData.data_rows[recordData.time_step -1 ][2] <  self.goal[servo][0]):
                    #     self.stat = 0

                self.get_time(time.time(), i, "finish")
                if self.stats:
                    i+=1
                print(i)
                ##wip code:
                # for i in range(len(self.DXL_IDs[servo])):
                #     self.toSigned32(
                #         self.angle_diff(self.goal, present[i], i, servo), servo
                #     )
                # if stat & () :
                #     i = i + 1
                #     stat=1
                # break

        except KeyboardInterrupt:
            print("Interrupted")
            for servo in self.servos:
                self.Disable(servo)
            return ""
        if recordData != "norecord":
            recordData.save_data(csv_path,1)
        self.get_seq(time.time(), str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
        print("loop completed ")

        # self.liveplot(str(datetime.datetime.now()).replace(" ", "_").replace(":", "-")+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn),seq_num)
        # self.Disable("pro")
        self.times = {}
        self.run = [1, 1, 1]
        print(
            "seq_num",
            seq_num,
            "---------------------------------------------------------",
        )

        return self
    
    def Automation_data(self,seq_num, recordData, samples,cycles,trails,Wipe_trj):
        # self.seq_num=seq_num
        for servo in self.servos:
            self.Enable(servo)
        self.stats = 1
        try:
            goal=self.goal
            print(goal)#[132474,48933,-149]
            for servo in self.servos:   
                if servo == "pro":
                    #,[132474,48933,-149],[164043,62878,17687],[246194,41472,44186],[140788,67811,50354],[159661,64144,-38188],[250000,26955,-1080]]
                    for ii,pro_ang in enumerate([Wipe_trj]):#[[242300,17500,0]]:
                        print(ii,pro_ang)
                        for trail in range(trails):
                            if recordData != "norecord":
                                # Create D;irectory
                                csv_path, image_path, image_path_side = recordData.create_datadir(seq_num)

                            pro_current_values=self.getj("pro")
                            initial_pro_angles=self.value2angle(pro_current_values,servo)
                            pro_current_values=self.signed(pro_current_values,servo)
                            print("present_pro_angles",initial_pro_angles)
                            run=1
                            # sin_step=0.008
                            present_position=self.sin_record_wipe(pro_current_values, pro_ang, samples,cycles,recordData,image_path,image_path_side, servo)                            
                            if recordData != "norecord":
                                recordData.save_data(csv_path)
                            # self.get_seq(time.time(), str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            print("trail completed ",trail)
                            seq_num+=1
                            print("Draw line")                            
                            if getch() != chr(0x20):
                                break
                        print("Change angle", ii)                            
                        if getch() != chr(0x20):
                            break


        except KeyboardInterrupt:
            print('Interrupted')
            for servo in self.servos:
                self.Disable(servo)
    
        return seq_num

    def Automation_data_C(self,seq_num, recordData, samples,cycles,trails,Wipe_trj,comments=0):
        for servo in self.servos:
            self.Enable(servo)
        self.stats = 1
        try:
            goal=self.goal
            print(goal)#[132474,48933,-149]
            for servo in self.servos:   
                if servo == "pro":
                    #,[132474,48933,-149],[164043,62878,17687],[246194,41472,44186],[140788,67811,50354],[159661,64144,-38188],[250000,26955,-1080]]
                    for ii,pro_ang in enumerate([Wipe_trj]):#[[242300,17500,0]]:
                        print(ii,pro_ang)
                        pro_current_values=self.getj("pro")
                        initial_pro_angles=self.value2angle(pro_current_values,"pro")
                        print("present_pro_angles",initial_pro_angles)
                        Wipe_init = [pro_current_values,pro_ang[0]]
                        present_position=self.sin_movej(Wipe_init[0],Wipe_init[1],0.02,"pro")
                        for trail in range(trails):
                            if recordData != "norecord":
                                # Create D;irectory
                                csv_path, image_path, image_path_side = recordData.create_datadir(seq_num)


                            run=1
                            # sin_step=0.008
                            present_position=self.sin_record_wipe(pro_ang[0], pro_ang[1:], samples,cycles,recordData,image_path,image_path_side, servo)                            
                            if recordData != "norecord":
                                recordData.save_data(csv_path,comments)
                            # self.get_seq(time.time(), str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            print("trail completed ",trail)
                            seq_num+=1
                            # print("Draw line")                            
                            # if getch() != chr(0x20):
                            #     break
                        # print("Change angle", ii)                            
                        # if getch() != chr(0x20):
                        #     break


        except KeyboardInterrupt:
            print('Interrupted')
            for servo in self.servos:
                self.Disable(servo)

        return seq_num


                    
            # Start Data Collection
