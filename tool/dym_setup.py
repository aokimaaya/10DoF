# from macpath import join
# "a" variable means angle
# "v" variable means value of servo

import datetime
import os
import time
import numpy as np
from dynamixel_sdk import *
# from utils.pixel import ArUcoDetector

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
        self.DEVICENAME = dict(L1 = "COM6", F1 = "COM3", L2 = "COM5", F2 = "COM7")  # list all comports
        self.servo_com = dict(
            L1 = ["mx1061", "xm540L1"],##[ "xm540L1"],###
            F1 = ["pro1", "xm540F1"],
            L2 = ["mx1062", "xm540L2"],
            F2 = ["pro2", "xm540F2"]
        )  # define leader follower, list all servo types
        self.DXL_IDs = dict(
            mx1061   = [1],
            xm540L1  = [2, 3, 4, 5], ## [1],###
            #xm430L1  = [5, 6, 7],
            pro1     = [0, 1, 2, 3],
            xm540F1  = [4, 5],
            #xm430F1  = [7],
            mx1062   = [6],
            xm540L2  = [7, 8, 9, 10],
            #xm430L2  = [12, 13, 14],
            pro2     = [11, 6, 7, 8],
            xm540F2  = [9, 10],
            #xm430F2  = [14],
        )
        self.prev_val = dict(
            mx1061   = [1],
            xm540L1  = [2, 3, 4, 5], ##[1],###
            #xm430L1  = [5, 6, 7],
            pro1     = [0, 1, 2, 3],
            xm540F1  = [4, 5],
            #xm430F1  = [7],
            mx1062   = [6],
            xm540L2  = [7, 8, 9, 10],
            #xm430L2  = [12, 13, 14],
            pro2     = [11, 6, 7, 8],
            xm540F2  = [9, 10],
            #xm430F2  = [14],
        )

        #self.aruco_detector = ArUcoDetector()

        self.PROTOCOL_VERSION = 2.0
        self.TORQUE_ENABLE    = 1
        self.TORQUE_DISABLE   = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 5
        self.portHandler = dict(
            L1 = PortHandler(self.DEVICENAME["L1"]), F1 = PortHandler(self.DEVICENAME["F1"]),
            L2 = PortHandler(self.DEVICENAME["L2"]), F2 = PortHandler(self.DEVICENAME["F2"])
        )
        self.stat = dict(
            mx1061={}, xm540L1={}, pro1={}, xm540F1={},
            mx1062={}, xm540L2={}, pro2={}, xm540F2={}
        )
        self.run      = [True, True, True, True, True, True, True, True, True, True,True, True]  # only once for while loop
        self.times    = {}
        self.timediff = {}
        # change !!
        self.servo_ranges = dict()
        angle_list        = range(-180, 180)
        theta_ranges      = [round(n * 11.375) for n in range(-180, 180)]
        self.servo_ranges["xm540L1"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm540F1"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm540L2"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm540F2"] = dict(zip(theta_ranges, angle_list))

        angle_list1   = range(-180, 180)
        theta_ranges1 = [round(n * 2788.461111111) for n in range(-180, 180)]
        self.servo_ranges["pro1"] = dict(zip(theta_ranges1, angle_list1))
        self.servo_ranges["pro2"] = dict(zip(theta_ranges1, angle_list1))

        """"
        angle_list   = range(-180, 180)
        theta_ranges = [round(n * 11.375) for n in range(-180, 180)]
        self.servo_ranges["xm430L1"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm430F1"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm430L2"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["xm430F2"] = dict(zip(theta_ranges, angle_list))
        """

        angle_list = range(-180, 180)
        theta_ranges = [round(n * 11.375) for n in range(-180, 180)]
        self.servo_ranges["mx1061"] = dict(zip(theta_ranges, angle_list))
        self.servo_ranges["mx1062"] = dict(zip(theta_ranges, angle_list))

        # angle_list2 = range(-501923,501923)
        # theta_ranges2 = [n*0.00035862 for n in range(-501923,501923)]

        # self.servo_ranges["pro"] = dict(zip(angle_list2, theta_ranges2))

        # angle_list1 = range(-2048,2048)
        # theta_ranges1 = [n*0.08789 for n in range(-2048,2048)]

        # self.servo_ranges["xm540"] = dict(zip(angle_list1, theta_ranges1))

        """
        For data collextion
        x1=xm540_present_values
        x2=xm540_filtered_values
        y1=pro_present_values
        y2=pro_filtered_values

        stat to identify direction
        """
        # Dynamixel Pro,xm540 Setting -> Base Joint, Shoulder Joint, Elbow Joint
        self.ADDR_TORQUE_ENABLE = dict(
            mx1061=64, xm540L1=64, pro1=512, xm540F1=64,
            mx1062=64, xm540L2=64, pro2=512, xm540F2=64
        )
        self.ADDR_GOAL_POSITION = dict(
            mx1061=116, xm540L1=116, pro1=564, xm540F1=116,
            mx1062=116, xm540L2=116, pro2=564, xm540F2=116
        )
        self.ADDR_PRESENT_POSITION = dict(
            mx1061=132, xm540L1=132, pro1=580, xm540F1=132,
            mx1062=132, xm540L2=132, pro2=580, xm540F2=132
        )
        self.DXL_MINIMUM_POSITION_VALUE = dict(
            mx1061=0, xm540L1=0, pro1=-501923, xm540F1=0,
            mx1062=0, xm540L2=0, pro2=-501923, xm540F2=0
        )
        self.DXL_MAXIMUM_POSITION_VALUE = dict(
            mx1061=4095, xm540L1=4095, pro1=501923, xm540F1=4095,
            mx1062=4095, xm540L2=4095, pro2=501923, xm540F2=4095
        )
        self.ADDR_Present_Current = dict(
            mx1061=126, xm540L1=126, pro1=574, xm540F1=126,
            mx1062=126, xm540L2=126, pro2=574, xm540F2=126
        )
        self.Threshold = dict(
            mx1061=5, xm540L1=5, pro1=20, xm540F1=5,
            mx1062=5, xm540L2=5, pro2=20, xm540F2=5
        )
        self.LEN_PRESENT_POSITION = dict(
            mx1061=4, xm540L1=4, pro1=4, xm540F1=4,
            mx1062=4, xm540L2=4, pro2=4, xm540F2=4
        )

        # self.ADDR_Current_Limit = dict(pro=38, xm540=4095)
        self.BAUDRATE = 2000000  #  # 57600 115200  #
        ##
        # check devices ports connectivity
                    
        self.bias = dict(
            mx1061=2048, xm540L1=2048, pro1=0, xm540F1=2048,
            mx1062=2048, xm540L2=2048, pro2=0, xm540F2=2048
        )  
        self.goal = dict(
            mx1061   = [2048], 
            xm540L1  = [2048, 2048, 2048, 2048],  
            pro1     = [132227, 132227, 48997, 500], 
            xm540F1  = [2048, 2048], 
            mx1062   = [2048], 
            xm540L2  = [2048, 2048, 2048, 2048],  
            pro2     = [132227, 132227, 48997, 500], 
            xm540F2  = [2048, 2048], 
        )
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
                print("Isport", servo)
                # if self.Enable(servo) == "COMM_SUCCESS":
                #     print(self.dxl_comm_result,servo,"port is usable")
                self.servos = self.servos + servo

        print("servos available:", self.servos)
        # for servo in self.servos:
        #     self.Disable(servo)
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

        self.packetHandlers  = {}
        self.reaches         = {}
        self.group_bulk_read = {}
        self.collection      = {}
        # self.present_angles = {}
        
        for servo in self.servos:
            print(servo)
            self.collection[servo] = {}
            self.reaches[servo]    = []
            # self.packetHandlers[servo]  = []
            self.packetHandlers[servo]  = PacketHandler(self.PROTOCOL_VERSION)
            # self.group_bulk_read[servo] = GroupBulkRead(self.portHandler[self.servos_LF(servo)], self.packetHandlers[servo])
            
            for i in range(len(self.DXL_IDs[servo])):
                # self.packetHandlers[servo].append(PacketHandler(self.PROTOCOL_VERSION))

                self.reaches[servo].append(1)
                self.collection[servo][i + 1] = []
                # # Add parameter storage for Dynamixel#1 present position
                # dxl_addparam_result = self.group_bulk_read[servo].addParam(
                #     self.DXL_IDs[servo][i], self.ADDR_PRESENT_POSITION[servo], self.LEN_PRESENT_POSITION[servo]
                #     )
                # if dxl_addparam_result != True:
                #     print("[ID:%03d] groupBulkRead addparam failed" % self.DXL_IDs[servo][i])
                #     quit()
                # print("Added parameter storage for bulk read: ",servo ,self.DXL_IDs[servo][i])
                # # self.present_angles[servo].append()
        # print("self.DXL_IDs",self.DXL_IDs,"self.packetHandlers",self.packetHandlers,"self.reaches",self.reaches)

    def servos_LF(self, servo):
        LF = list(self.servo_com.values())
        # print("LF",LF)
        for i, ser_ls in enumerate(LF):
            # print((self.servo_com.keys()))
            if servo in ser_ls:
                return list(self.servo_com.keys())[i]

    def portIsUsable(self, portName):
        import serial
        from serial import SerialException

        # print(portName)
        try:
            ser = serial.Serial(port=portName)
            print(portName, True)
            return True

        except:
            print(False)
            return False

    # def IfDevice(self,servo):
    #     # Get Dynamixel model number
    #     for i in range(len(self.DXL_IDs[servo])):
    #         dxl_model_number, dxl_comm_result, dxl_error = self.packetHandlers[servo].ping(portHandler, DXL_ID)
    #         if dxl_comm_result != COMM_SUCCESS:
    #             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         elif dxl_error != 0:
    #             print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         else:
    #             print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))

    # def getj_bulk(self, servo):
    #     # Check if groupbulkread data of Dynamixel#1 is available
    #     # print("self.group_bulk_read[servo]")
    #     dxl_comm_result = self.group_bulk_read[servo].txRxPacket()
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % self.packetHandlers[servo].getTxRxResult(dxl_comm_result),dxl_comm_result,servo)

    #     self.present_bulk_angles = []
    #     start = time.time()
    #     for i in range(len(self.DXL_IDs[servo])):
    #         # time.sleep(0.01)

    #         dxl_getdata_result = self.group_bulk_read[servo].isAvailable(self.DXL_IDs[servo][i], self.ADDR_PRESENT_POSITION[servo], self.LEN_PRESENT_POSITION[servo])
    #         if dxl_getdata_result != True:
    #             print("[ID:%03d] groupBulkRead getdata failed" % self.DXL_IDs[servo][i])
    #             print("read bulk error: try again !!! Press esc!!!",dxl_getdata_result,servo,i)
    #             if getch() == chr(0x1B):
    #                 for servo in self.servos:
    #                     self.Disable(servo)
    #                 quit()

    #         self.present_bulk_angles.append(self.group_bulk_read[servo].getData(self.DXL_IDs[servo][i], self.ADDR_PRESENT_POSITION[servo], self.LEN_PRESENT_POSITION[servo]))
    #     end = time.time()
    #     print("time " , end-start)
    #     print("self.present_bulk_angles", servo, self.present_bulk_angles)
    #     return self.present_bulk_angles

    # def bulk_clear(self):
    #     # Clear bulkread parameter storage
    #     for servo in self.servos:
    #         self.group_bulk_read[servo].clearParam()
    #     print("clear bulk read")
    def getj(self, servo):
        MAX_RETRIES = 3
        # self.present_angles = []

        self.present_angles = [0]*len(self.DXL_IDs[servo])
        # print(self.present_angles,self.DXL_IDs[servo],servo,len(self.DXL_IDs[servo]))
        for i in range(len(self.DXL_IDs[servo])):
            retries = 0
            # print(self.servos_LF(servo),self.packetHandlers[servo])
            while retries < MAX_RETRIES:
                (
                    self.dxl_present_position,
                    self.dxl_comm_result,
                    self.dxl_error,
                ) = self.packetHandlers[servo].read4ByteTxRx(
                    self.portHandler[self.servos_LF(servo)],
                    self.DXL_IDs[servo][i],
                    self.ADDR_PRESENT_POSITION[servo],
                )
                self.present_angles[i]=self.dxl_present_position
                # print("self.present_angles",self.present_angles)
                if self.dxl_comm_result != COMM_SUCCESS:
                    print(
                        "%s"
                        % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result),
                         self.dxl_comm_result, COMM_SUCCESS, servo,i
                    )
                    print("Communication error: Retrying...press space")
                    # if getch() == chr(0x1B):
                    #     # for servo in self.servos:
                    #     #     self.Disable(servo)
                    #     quit()
                    delay_factor = retries 
                    delay_time = min(0.001 * delay_factor, 0.002)  # Maximum delay 0.5 seconds
                    if retries>0:
                        time.sleep(delay_time)
                    print("delay_time,retries",delay_time,retries)
                    retries += 1
                    if retries > 2:
                        print("self.prev_val[servo]",self.prev_val[servo],retries)

                        self.present_angles[i] = self.prev_val[servo][i]

                        
                        print("read error: try again !!! Press esc!!!")
                    
                        if getch() == chr(0x1B):
                            # for servo in self.servos:
                            #     self.Disable(servo)
                            quit()
                        else:
                            break
                elif self.dxl_error != 0:
                    print(
                        "%s"
                        % self.packetHandlers[servo].getRxPacketError(self.dxl_error),
                         servo,i
                    )
                    # Handle other types of errors
                    # break
                else:
                    break
                
                
        # if retries > 2:
        #     print("Max retries reached. Utilizing previous value...")
        #     self.present_angles = self.prev_val[servo]
        #         # Implement further actions or recovery mechanisms
        #         # ...
        #     return self.present_angles
        # else:
        self.prev_val[servo] = self.present_angles
        return self.present_angles


    def getj_old(self, servo):
        self.present_angles = []
        
        for i in range(len(self.DXL_IDs[servo])):
            (
                self.dxl_present_position,
                self.dxl_comm_result,
                self.dxl_error,
            ) = self.packetHandlers[servo].read4ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_PRESENT_POSITION[servo],
            )
            self.present_angles.append(self.dxl_present_position)
            # print("getj",self.present_angles)
            ##debug for read error

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result),
                    i,self.dxl_comm_result,COMM_SUCCESS
                )
                print("read error: try again !!! Press esc!!!",servo,i)
                if getch() == chr(0x1B):
                    # for servo in self.servos:
                    #     self.Disable(servo)
                    quit()
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo].getRxPacketError(self.dxl_error),
                    i,
                )

        # print("self.present_angles-getj",servo,self.present_angles)
        return self.present_angles

    def getCurrent(self, servo):
        self.current = []
        for i in range(len(self.DXL_IDs[servo])):
            (
                self.dxl_present_current,
                self.dxl_comm_result,
                self.dxl_error,
            ) = self.packetHandlers[servo].read2ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_Present_Current[servo],
            )
            if self.dxl_present_current > 0x7FFF:
                self.dxl_present_current = self.dxl_present_current - 65536

            self.current.append(self.dxl_present_current)
            # print(self.current)
            ##debug for read error

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result),servo,i
                )
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo].getRxPacketError(self.dxl_error),servo,i
                )

        # print(joint_angles)
        return self.current

    def movej(self, goal, servo) -> None:
        self.diff = [1] * len(self.DXL_IDs[servo])
        # time.sleep(0.002)
        for i in range(len(self.DXL_IDs[servo])):
            # print("move-i",i)
            # print("move_j",servo,self.DXL_IDs[servo][i],goal[servo][i],"self.diff[i]",self.diff[i])
            # if not self.diff:
            #     self.diff = [1] * len(self.DXL_IDs[servo])

            if self.diff[i]:

                self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo].write4ByteTxRx(
                    self.portHandler[self.servos_LF(servo)],
                    self.DXL_IDs[servo][i],
                    self.ADDR_GOAL_POSITION[servo],
                    int(goal[servo][i]),
                )
                ##debug for read error

                if self.dxl_comm_result != COMM_SUCCESS:
                    print(
                        "%s"
                        % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result),servo,i
                    )
                    if getch() == chr(0x1B):
                        # for servo in self.servos:
                        #     self.Disable(servo)
                        quit()
                elif self.dxl_error != 0:
                    print(
                        "%s"
                        % self.packetHandlers[servo].getRxPacketError(self.dxl_error),servo,i
                    )

            if self.dxl_comm_result != COMM_SUCCESS:
                print(
                    "%s"
                    % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result),servo,i
                )
            elif self.dxl_error != 0:
                print(
                    "%s"
                    % self.packetHandlers[servo].getRxPacketError(self.dxl_error),servo,i
                )

    def step_movej(self, goal, servo, step) -> None:
        """
        # Increment the servo value with constant steps
        """
        if "pro" in servo:
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
                self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo].write4ByteTxRx(
                    self.portHandler[self.servos_LF(servo)],
                    self.DXL_IDs[servo][i],
                    self.ADDR_GOAL_POSITION[servo],
                    goal_pro,
                )
                # self.reach(target,servo)

    def all_com_list(self, ls, val):
        self.diff = [1] * len(ls)
        for i, v in enumerate(ls):
            if int(abs(v)) < val:
                self.diff[i] = 0

        return any(self.diff)

    def sin_movej(self, initial_values, Final_values, sin_step, L_F):
        ang_values         = {}
        angle_displacement = {}
        step_size          = {}
        min_ang_dis        = {}
        print("Final_values", Final_values)
        for servo in self.servo_com[L_F]:
            # if "L" in L_F or "pro" not in servo:
            #     sin_step=sin_step*1.5 ## increse the speed for small servos
            initial_values_ = self.signed(initial_values[servo], servo)
            Final_values_   = self.signed(Final_values[servo], servo)
            angle_displacement[servo] = np.array(initial_values_) - np.array(Final_values_)
            step_size[servo] = [0] * len(initial_values_)
            # sin_step=0.01
            min_ang_dis[servo] = max(angle_displacement[servo])
            print("angle_displacement " +servo ,angle_displacement[servo])
            if self.all_com_list(angle_displacement[servo], self.Threshold[servo]):

                for tta in np.arange(0, np.pi, sin_step):
                    # for servo in self.servo_com[L_F]:
                    runz = 1
                    step_size[servo] = (
                        step_size[servo] + (angle_displacement[servo] / 2) * np.sin(tta) * sin_step
                    )  # * (angle_displacement[servo]/min_ang_dis[servo] )
                    ang_values[servo] = initial_values_ - step_size[servo]
                    
                    self.goal[servo] = ang_values[servo]
                    self.movej(self.goal, servo)
                    while runz:
                        runz = self.reach(self.goal, servo)
                        # run=0
                    print(servo, ang_values[servo])
            else:
                print(servo, " - No angle change")
        # print("initial_pro_angles", initial_values)#self.value2angle(initial_values,servo))
        # print("Final_angle_values", Final_values)#self.value2angle(Final_values, servo))
        print("angle_displacement", angle_displacement)

        return ang_values
    
    # Modified sin_movej function to handle XM540 servo values correctly and reduce unnecessary rotations
    def sin_movej_ex(self, initial_values, Final_values, sin_step, L_F):
        ang_values = {}
        angle_displacement = {}
        step_size = {}
        min_ang_dis = {}
        print("Final_values", Final_values)

        for servo in self.servo_com[L_F]:
            if "pro" in servo:
                # Original handling for Pro servos
                initial_values_ = self.signed(initial_values[servo], servo)
                Final_values_ = self.signed(Final_values[servo], servo)
                angle_displacement[servo] = np.array(initial_values_) - np.array(Final_values_)
            elif "xm540" in servo:
                # Modified handling for XM540 to find the shortest path
                initial_values_ = self.signed(initial_values[servo], servo)
                Final_values_ = self.signed(Final_values[servo], servo)
                # Calculate direct and wrapped angle displacement
                direct_displacement = np.array(Final_values_) - np.array(initial_values_)
                wrapped_displacement = ((np.array(Final_values_) - np.array(initial_values_)) + 4096) % 4096
                print("direct_displacement", direct_displacement, "wrapped_displacement", wrapped_displacement)
                # Choose the smaller displacement in terms of magnitude
                if abs(direct_displacement) < abs(wrapped_displacement):
                    angle_displacement[servo] = direct_displacement
                else:
                    angle_displacement[servo] = wrapped_displacement

            step_size[servo] = [0] * len(initial_values_)
            min_ang_dis[servo] = max(abs(angle_displacement[servo]))

            print("angle_displacement " + servo, angle_displacement[servo])

            if self.all_com_list(angle_displacement[servo], self.Threshold[servo]):
                for tta in np.arange(0, np.pi, sin_step):
                    runz = 1
                    step_size[servo] = (
                        step_size[servo] + (angle_displacement[servo] / 2) * np.sin(tta) * sin_step
                    )
                    ang_values[servo] = initial_values_ + step_size[servo]
                    self.goal[servo] = ang_values[servo]
                    self.movej(self.goal, servo)

                    while runz:
                        runz = self.reach(self.goal, servo)
                    print(servo, ang_values[servo])
            else:
                print(servo, " - No angle change")

        print("angle_displacement", angle_displacement)
        return ang_values


    def sin_movej_se(self, initial_values, Final_values, sin_step, L_F):
        ang_values         = {}
        angle_displacement = {}
        step_size          = {}
        min_ang_dis        = {}
        for servo in self.servo_com[L_F]:
            # initial_values = self.signed(initial_values, servo)
            # Final_values = self.signed(Final_values, servo)
            angle_displacement[servo] = np.array(initial_values[servo]) - np.array(Final_values[servo])
            # print("angle_displacement", angle_displacement[servo])
            step_size[servo] = [0] * len(initial_values[servo])
            # sin_step=0.01
            min_ang_dis[servo] = max(angle_displacement[servo])  
            # , key=angle_displacement[servo].get)#min(angle_displacement[servo])
            # print("angle_displacement " +servo ,angle_displacement[servo])
        servo_list_ = dict(
            sorted(min_ang_dis.items(), key = lambda item: abs(item[1]), reverse = True)).keys()
        for tta in np.arange(0, np.pi, sin_step):
            for servo in servo_list_:  # self.servo_com[L_F]:
                # if angle_displacement[servo] < 2:
                #     continue
                # else:
                run = 1
                step_size[servo] = (
                    step_size[servo] + (angle_displacement[servo] / 2) * np.sin(tta) * sin_step
                )  # * (abs(min_ang_dis[servo] - angle_displacement[servo])/ angle_displacement[servo])

                ang_values[servo] = initial_values[servo] - step_size[servo]

                self.goal[servo] = ang_values[servo]
                self.movej(self.goal, servo)
                while run:
                    run = self.reach(self.goal, servo)
                    # run=0
                print("pro_angles", ang_values[servo])
        # print("initial_pro_angles", initial_values)#self.value2angle(initial_values,servo))
        # print("Final_angle_values", Final_values)#self.value2angle(Final_values, servo))

        return ang_values

    def angle_diff(self, goal, present, i, servo):

        if "pro" in servo:
            present_ = self.toSigned32(present, servo)
            # print("present",present_,goal[servo],present_ >= goal[servo][i],i)
            if present_ >= goal[servo][i]:
                self.stat[servo][self.DXL_IDs[servo][i]] = 0
            else:
                self.stat[servo][self.DXL_IDs[servo][i]] = 1
            # elif servo=="xm540":
            #     self.stat[servo][self.DXL_IDs[servo][i]]=1
        return present_  # self.stat[servo][self.DXL_IDs[servo]]

    def sin_record_wipe(
            self, 
            initial_values, 
            Final_values, 
            samples, 
            cycles, 
            recordData, 
            image_path, 
            image_path_side, 
            B_hover_pos = [227311, -2708, 241532], 
            servo = "pro",
        ):
        """
        ## Record for sin_movej function
        """

        ang_values = []
        initial_o_values = self.signed(initial_values, servo)
        for cyc in range(cycles):
            initial_values = initial_o_values
            print("cycle", cyc)
            pro_current_values = self.getj("pro")
            print("present_pro_angles", cyc, pro_current_values)

            for final_value in Final_values:

                angle_displacement = np.array(initial_values) - np.array(final_value)
                print("initial_values,final_value", initial_values, final_value)
                print("angle_displacement", angle_displacement)

                step_size = [0, 0, 0]
                sin_step  = np.pi / (samples / len(Final_values))
                samp_len  = np.arange(0, np.pi, sin_step)
                print("len(samp_len)", len(samp_len), "sin_step", sin_step)
                i = 0

                while i < len(samp_len):
                    run              = 1
                    step_size        = (step_size + (angle_displacement / 2) * np.sin(samp_len[i]) * sin_step)
                    ang_values       = initial_values - step_size
                    self.goal[servo] = ang_values
                    self.movej(self.goal, servo)

                    if recordData != "norecord":
                        recordData.record(image_path, image_path_side, "pro")
                    while run:
                        run = self.reach(self.goal, servo)
                    i += 1
                initial_values = final_value
                print("initial_pro_angles", initial_values)

        return ang_values

    def Wipe_Generator(self, Wipe_Angles, step):
        """
        #mainly for generating wipe positions in the middle from wipe_angles[0] to wipe_angles[1]
        """
        Wipe_C = []
        for x1, y1 in zip(Wipe_Angles[0], Wipe_Angles[1]):
            print(x1, y1)
            A = [[], [], []]

            for i, q in enumerate(A):
                diff = x1[i] - y1[i]
                if diff > 0:
                    for j, x in enumerate(
                        range(0, diff + round(diff / step), round(diff / step))
                    ):
                        A[i].append(y1[i] + x)
                else:
                    for j, x in enumerate(
                        range(diff, round(abs(diff) / step), round(abs(diff) / step))
                    ):
                        A[i].append(x1[i] - x)

            Wipe_C.append(list(zip(A[0], A[1], A[2])))
        return list(zip(*Wipe_C))

    def Enable(self, servo):
        for i in range(len(self.DXL_IDs[servo])):
            print(servo, i, self.DXL_IDs[servo])
            self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo].write1ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_TORQUE_ENABLE[servo],
                self.TORQUE_ENABLE,
            )
            print("Enable " + servo, self.DXL_IDs[servo][i], self.ADDR_TORQUE_ENABLE[servo],)
            if self.dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result), i,)
            elif self.dxl_error != 0:
                print("%s" % self.packetHandlers[servo].getRxPacketError(self.dxl_error), i,)
        return self.dxl_comm_result

    def Disable(self, servo):
        for i in range(len(self.DXL_IDs[servo])):
            self.dxl_comm_result, self.dxl_error = self.packetHandlers[servo].write1ByteTxRx(
                self.portHandler[self.servos_LF(servo)],
                self.DXL_IDs[servo][i],
                self.ADDR_TORQUE_ENABLE[servo],
                self.TORQUE_DISABLE,
            )
        print("Disable " + servo)
        if self.dxl_comm_result != COMM_SUCCESS:
            print(
                "%s" % self.packetHandlers[servo].getTxRxResult(self.dxl_comm_result)
            )
        elif self.dxl_error != 0:
            print("%s" % self.packetHandlers[servo].getRxPacketError(self.dxl_error))

    def value2angle(self, dxl, servo):
        """
        # mapping from servo position value to angle
        """
        d      = self.servo_ranges[servo]
        prepos = []
        for ang in dxl:
            ang = self.toSigned32(ang, servo) - self.bias[servo]
            # print("ang",ang)
            prepos.append(
                list(d.values())[
                    list(d.keys()).index(min(d.keys(), key=lambda x: abs(x - ang)))
                ]
            )

        return prepos

    # def convertLF(self,dxl,servo):
    """ Work in progress
        Will be used in combining 6dof & 3dof code.
        Alternative for round error. Function idea as an universal range converter
    """
    #    Shoudl
    #     d=self.servo_ranges[servo]
    #     # dxl=self.value2angle(dxl,servo)
    #     prepos = []
    #     for ang in dxl:
    #         # ang= self.toSigned32(ang,servo)
    #         prepos.append(list(d.keys())[list(d.values()).index(min(d.values(), key=lambda x:abs(x-ang)))] + self.bias[servo])

    #     return prepos

    def reach(self, goal, servo):
        """
        Help to identify if all the 3 motors are reached the goal position
        """
        # print("reach getj")
        values = self.getj(servo)

        if "pro" in servo:

            for i in range(len(self.DXL_IDs[servo])):
                degre = self.toSigned32(values[i], servo)
                goal[servo][i] = self.toSigned32(goal[servo][i], servo)

                # print(
                #     "[PROID:%03d] GoalPos:%03d AnglePos:%03d "
                #     % (self.DXL_IDs[servo][i], goal[servo][i], degre)
                # )  # , self.value2angle(d,values[i])))degree PresentPos:%03d
                print(self.reaches[servo], goal[servo][i] - degre)
                if not (abs(goal[servo][i] - degre) > 100):
                    self.reaches[servo][i] = 0

                    # return self.reaches[servo][0] or self.reaches[servo][1] or self.reaches[servo][2]
        else:#if servo in ["mx106", "xm540L", "xm430L", "xm540F", "xm430F"]:
            # degre = self.value2angle(values, servo)
            # print(
            #     self.value2angle(values, "xm540"),
            #     values,
            #     self.DXL_IDs[servo],
            #     goal[servo],
            #     self.value2angle(goal[servo], servo),
            # )
            for i in range(len(self.DXL_IDs[servo])):  # len(self.DXL_IDs[servo])
                # print(i)
                # if values[i]%10 ==0:
                # value=self.toSigned32(values[i])
                print(
                    "["+servo+":%03d] GoalPos:%03d PresentPos:%03d"
                    % (self.DXL_IDs[servo][i], goal[servo][i], values[i])
                )
                # print("reaching" ,i, self.reaches[servo],goal[servo][i] - values[i])

                if not (
                    abs(goal[servo][i] - values[i]) > 2  # self.DXL_MOVING_STATUS_THRESHOLD
                ):
                    self.reaches[servo][i] = 0

        return any(self.reaches[servo])

    # def toSigned32(self, n, servo="pro"):
    #     #convert unsigned to signed
    #     if  "pro" in servo:
    #         # print(n)
    #         n = int(n) & 0xFFFFFFFF
    #         # if n >= 501923:
    #         #     print("n",n , (n | (-(n & 0x80000000))) )
    #         return n | (-(n & 0x80000000))
    #     # if "xm540" in servo:
    #     #     if n > 0x7FFF:
    #     #         return n - 65536
    #     else:
    #         return n
    def toSigned32(self,n, servo="pro", mode="position"):
        # Convert unsigned to signed for different servo types and modes
        print("n", n, servo, mode)
        if "pro" in servo:
            n = int(n) & 0xFFFFFFFF
            # Handle 32-bit signed conversion for Pro servos
            if n & 0x80000000:
                return n - 0x100000000
            else:
                return n #| (-(n & 0x80000000))
        elif "xm540" in servo:
            # XM540 servos have different modes
            n = int(n)
            if mode == "position":
                # Position Control Mode: Range is 0-4095 or -2048 to 2048
                n = n % 4096  # Ensure n is within 0-4095
                # if n > 2047:
                #     return n - 4096
                # else:
                return n
            # elif mode in ["extended_position", "current_based_position"]:
            #     # Extended Position Control Mode: Range is -1,048,575 to 1,048,575
            #     n = n & 0x1FFFFF  # Ensure n is within 21-bit range
            #     if n & 0x100000:  # If the 21st bit is set, it's negative
            #         return n - 0x200000  # Convert to signed by subtracting 2^21
            #     else:
            #         return n
        else:
            return n

    def signed(self, actual_values, servo):
        """Convert to signed int32 for all the servo values"""
        self.sign_values = []
        for id in range(len(self.DXL_IDs[servo])):
            self.sign_values.append(self.toSigned32(actual_values[id], servo))
        return self.sign_values

    def LiveLPF_initialize(self, d=2, fs=30, Wn=2):
        """
        ##Live low pass filter parameters initialization
        """
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
                    d, Wn = Wn, fs = fs, btype = "low", ftype = "butter", output = "sos"
                )
                # print(self.sos)
                self.live_sosfilter[servo][self.DXL_IDs[servo][id]] = LiveSosFilter(
                    self.sos[servo][self.DXL_IDs[servo][id]]
                )
                print("self.goal[servo][id]", self.goal[servo][id],servo, id)
        return self.live_sosfilter

    def Filter(self, joint_angles, servo):
        """Live filter with fs as sampling frequency from initalization"""
        self.filter_angles = []  # dict(xm = [],pro = [])
        for i, id in enumerate(self.DXL_IDs[servo]):
            id=id-1
            if len(self.DXL_IDs[servo]) == len(joint_angles):
                y=joint_angles[i]
            else:
                print("filter error")
                if getch() != chr(0x20):
                    # for servo in self.servos:
                    #     self.Disable(servo)
                    quit()
            # print(id)
            # if "pro" in servo:
            filter_value = self.live_sosfilter[servo][self.DXL_IDs[servo][i]](y)
            if self.run[id]:
                print("Inside filter--------------------",servo,id)
                # time.sleep(0.25)
                while self.run[id]:

                    filter_value = self.live_sosfilter[servo][
                        self.DXL_IDs[servo][i]
                    ](y)
                    print("filter", y - filter_value)
                    if abs(y - filter_value) < 3:
                        self.run[id] = False
                self.run[id] = False

            self.filter_angles.append(filter_value)
            # else:
            #     self.filter_angles.append(
            #         self.live_sosfilter[servo][self.DXL_IDs[servo][id]](y)
            #     )

        return self.filter_angles

    def valuesforplot(self, time, y1, y2, servo):
        """Collecting values for plot
        x1=xm_present_values
        x2=xm_filtered_values
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
                    plt.figure(figsize = [128, 80])
                    t = [fmt(g - time[0]) for g in time]  # np.arange(0, len(time), 1)
                    # print(j,time,t)
                    plt.plot(t, data, lw = 2, label = "Noisy signal")
                    plt.scatter(t, data, lw = 2, label = "Noisy signal")
                    # plt.plot(time, y_live_sosfilt, lw=2, label="SciPy lfilter")
                    plt.plot(
                        t,
                        y_live_sosfilt,
                        lw = 3,
                        label = "LiveLFilter-" + name + "-" + i + "-" + str(j),
                    ) 

                    plt.legend(
                        loc = "lower center",
                        bbox_to_anchor = [0.5, 1],
                        ncol = 2,
                        fontsize = "smaller",
                    )
                    plt.scatter(
                        t,
                        y_live_sosfilt,
                        lw = 3,
                        label = "LiveLFilter-" + name + "-" + i + "-" + str(j),
                    )
                    plt.xticks(t, rotation = 90)
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
        """
        !Important Function!
        Converts the range of input values to desired "servo" output name.
        """
        if "pro" in servo :
            self.round_a = []
            for v in values:
                div = multiplier / 245.0 # just for having additional mutiplier 
                val = round((v - self.bias["xm540L2"]) / div)
                # print("v",v,val)
                val = val * multiplier
                self.round_a.append(val)
            return self.round_a
        else:
            self.round_a = []
            for v in values:
                div = multiplier / 245.0
                # print("v",v)
                val = round(self.toSigned32(v, "pro2") / div)  
                val = val / multiplier
                self.round_a.append(val + self.bias["xm540L2"])
                print("self.toSigned32(v)", v, self.toSigned32(v, "pro"),self.round_a)
            return self.round_a

    def get_seq(self, times, name):
        """Save the data as json files"""
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
        """Save times of every step
        Used for calculating the delay or sampling frequency.
        """
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

    # def get_output_values(self, input_values,L_F):
    #     output_values = {}

    #     for input_servo, input_values in input_values.items():
    #         if input_servo in self.DXL_IDs:

    #             servo_ids = self.DXL_IDs[input_servo]
    #             rounded_values = self.rounderror(input_values, 245.0, input_servo)
    #             if L_F
    #             for servo_type in self.DXL_IDs.items() :
    #                 if servo_type in self.servo_com[L_F]:
    #                     output_values[servo_type] = rounded_values[:len(servo_ids)]

    #     return output_values
    #########################################################################################
    # Control Code for dynamixel
    #########################################################################################
    # def equation(x):
    #     return (465 * x - 2980185) / -744

    def Initialize(self, Wipe_trj, sin_step=0.005):
        """
        #Intitialize the Leader to the same angle orientation as Follower. This is to avoid jerky jump of follower.
        """
        ## Enable Servo Motors
        for servo in self.servos:
            self.Enable(servo)

        try:
            F1_present_values = {}
            F1_current_values = {}
            L1_current_values = {}
            L1_present_values = {}
            F2_present_values = {}
            L2_current_values = {}
            L2_present_values = {}
            # print(F_present_values)
            ###################### initialize
            for servo in self.servo_com["F1"]:                                                    
                F1_present_values[servo] = self.signed(self.getj(servo),servo)
                print(servo, F1_present_values)
            for servo in self.servo_com["L1"]:  
                L1_present_values[servo] = self.signed(self.getj(servo),servo)
                print(servo, L1_present_values)
            for servo in self.servo_com["F2"]:                                                    
                F2_present_values[servo] = self.signed(self.getj(servo),servo)
                print(servo, F2_present_values)
            for servo in self.servo_com["L2"]:  
                L2_present_values[servo] = self.signed(self.getj(servo),servo)
                print(servo, L2_present_values)
            
            ### initial positions
            for L_F in ["F1"]:
                # if L_F == "L":
                for servo in self.servo_com[L_F]:
                    # print("L_Fgetj",L_F)
                    L1_current_values[servo] = self.signed(self.getj(servo),servo)
                # print(L_current_values)
                # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                Wipe_C_one = [L1_current_values] #+ [Wipe_trj[0][L_F]]
                Wlen       = len(Wipe_C_one) - 1
                # print("Wipe_C_one",Wipe_C_one)
                # print("sin motion")
                # if getch() == chr(0x1B):
                #     pass
                for w in range(Wlen):
                    print(w,Wipe_C_one)
                    # w = Wlen - w - 1
                    # print(w,w+1,Wipe_C_one[w], Wipe_C_one[w + 1])
                    present_position1 = self.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                    print("Enter", w / 2)
                    if w % 2 == 0:
                        print("Change angle of F1")
                    if getch() == chr(0x1B):
                        # for servo in DymControl.servos:
                        #     DymControl.Disa50ble(servo)
                        quit()
            for L_F in ["F2"]:
                # if L_F == "L":
                for servo in self.servo_com[L_F]:
                    # print("L_Fgetj",L_F)
                    L2_current_values[servo] = self.signed(self.getj(servo),servo)
                # print(L_current_values)
                # pro_current_values=dict(pro=DymControl.getj("pro"),xm430F= DymControl.getj("xm430F"),xm540=DymControl.getj("xm540"))
                Wipe_C_one = [L2_current_values] #+ [Wipe_trj[0][L_F]]
                Wlen       = len(Wipe_C_one) - 1
                # print("Wipe_C_one",Wipe_C_one)
                # print("sin motion")
                # if getch() == chr(0x1B):
                #     pass
                for w in range(Wlen):
                    print(w,Wipe_C_one)
                    # w = Wlen - w - 1
                    # print(w,w+1,Wipe_C_one[w], Wipe_C_one[w + 1])
                    present_position2 = self.sin_movej(Wipe_C_one[w], Wipe_C_one[w + 1], sin_step, L_F)
                    print("Enter", w / 2)
                    if w % 2 == 0:
                        print("Change angle of F2")
                    if getch() == chr(0x1B):
                        # for servo in DymControl.servos:
                        #     DymControl.Disa50ble(servo)
                        quit()
                # if L_F == "F":
            

            ### follower to leader
            L1_current_values      = self.rounderror(F1_present_values["pro1"], 245, "mx1061")
            xm540L1_present_value  = L1_current_values[1:] + F1_present_values["xm540F1"][:1]              
            xm430L1_present_values = F1_present_values["xm540F1"][1:]+[-0.983* F1_present_values["xm430F1"][0] + 5001.863]    #[int((465 * F_present_values["xm430F"][0] - 2980185) / -744)]
            
            L1_new_values = dict(mx1061 = L1_current_values[:1], xm540L1 = xm540L1_present_value , xm430L1 = xm430L1_present_values)

            print(F1_present_values , L1_current_values)
            print("L1_present_values", L1_present_values)
            print("L1_new_values", L1_new_values)
            present_position1 = self.sin_movej(L1_present_values, L1_new_values, sin_step, "L1")

            L2_current_values      = self.rounderror(F2_present_values["pro2"], 245, "mx1062")
            xm540L2_present_value  = L2_current_values[1:] + F2_present_values["xm540F2"][:1]              
            xm430L2_present_values = F2_present_values["xm540F2"][1:]+[-0.983* F2_present_values["xm430F2"][0] + 5001.863]    #[int((465 * F_present_values["xm430F"][0] - 2980185) / -744)]
            
            L2_new_values = dict(mx1062 = L2_current_values[:1], xm540L2 = xm540L2_present_value , xm430L2 = xm430L2_present_values)

            print(F2_present_values , L2_current_values)
            print("L2_present_values", L2_present_values)
            print("L2_new_values", L2_new_values)
            present_position = self.sin_movej(L2_present_values, L2_new_values, sin_step, "L2")
        ######################
            # wipe_one=[]
            # for Wipe_ in Wipe_trj:
            #     Leader_align_angles = self.rounderror(Wipe_["pro"], 245.0, "xm540")
            #     print(Leader_align_angles, "Leader_align_angles")
            #     print(Wipe_,
            #         "Leader_align_angles",
            #         self.value2angle(Leader_align_angles, "xm540"),
            #         "pro_angles",
            #         self.value2angle(F_present_values["pro"], "pro"),
            #     )
    
            #       # self.get_output_values(F_present_values,"L")#
            #     wipe_one.append(dict(
            #         xm540=[Leader_align_angles[2]],
            #         mx106=Leader_align_angles[:2],
            #         pro=Wipe_["pro"],
            #     ))
            # self.wipe_angle = Wipe_trj["pro"]
            # print(wipe_one)

        except KeyboardInterrupt:
            print('Interrupted')
            # for servo in self.servos:
            #     self.Disable(servo)

            return ""
        
    def generate_line_equation(self, x1, y1, x2, y2):
        # Calculate slope
        slope = (y2 - y1) / (x2 - x1)

        # Calculate y-intercept (c) using y = mx + c rearranged as c = y - mx
        c = y1 - slope * x1
        
        # Define the function for the line equation
        def line_function(x):
            return slope * x + c
        
        return line_function
    
    def generate_line_equation_y(self, x1, y1, x2, y2):
        # Calculate slope
        slope = (y2 - y1) / (x2 - x1)

        # Calculate y-intercept (c) using y = mx + c rearranged as c = y - mx
        c = y1 - slope * x1
        
        # Define the function for the line equation
        def line_function(y):
            return (y-c)/slope
        
        return line_function
    
    def LeaderFollowerRecord(self, seq_num, recordData, samples):
        """
        Control code as follows: edited for new 3dof
        - Read from the Leader
        - Convert Leader angle(value) to Follower angle(value)
        - Smooth the follower angles with low pass filter
        - Collect the plot values (only if recordData != "norecord")
        - Move the follower to the filtered target angles
        - record image and motor data (only if recordData != "norecord")
        """

        ##@ Initialize Low pass filter
        d  = 2
        fs = 20
        Wn = 2.0
        self.LiveLPF_initialize(d = d, fs = fs, Wn = Wn)
        ##
        L_present_values = {}
        L_current_values = {}
        F_present        = {}
        L_present        = {}
        target_values    = {}
        filtered_values  = {}
        
        # while 1:
        #     sensitivity = input( ## for 1:1 245.0 , but for better control use <0.01
        #         """Input sensitivity for control 
        #         sensitivity : """
        #     )
        #     if sensitivity != "":
        #         break

        sensitivity = 0.000001                  
        # Enable follower
        for servo in self.servo_com["F1"]:             
            self.Enable(servo)
        for servo in self.servo_com["F2"]:                   
            self.Enable(servo)
        
        ## check if record true
        if recordData != "norecord":
            # Create Directory
            print("Directory created")
            csv_path, image_path, image_path_side,excel_path = recordData.create_datadir(seq_num)#

        self.stats = 1 # variable to decide to start record only if the change in servo angle dectected, skip if "norecord"
        ### Capture Aruco pixel data before wiping with LF
        #self.aruco_detector.capture_frames(image_path_side,recordData, 'start', angle=slope)  # Capture "before" frames with angle 45
        B_hover_pos  = {}#dict(pro=[-17996, 53421, 214471]

        print("Enter to Start Leader Follower Control")
        
        # Generate line equation     # Input min and max
        # L1
        # x1, y1 = 
        # x2, y2 = 
        # line_eq1 = self.generate_line_equation(x1, y1, x2, y2)
        # L2
        # x1, y1 = 2373, 2165 
        # x2, y2 = 2728, 2139
        # line_eq1 = self.generate_line_equation(x1, y1, x2, y2)
        # x3, y3 = 2382, 2685
        # x4, y4 = 2839, 2094 
        # # line_eq2 = self.generate_line_equation(x3, y3, x4, y4)
        # x1, y1 = 2373, 2165 
        # x2, y2 = 2700, 1611
        # line_eq1 = self.generate_line_equation(x1, y1, x2, y2)
        # x3, y3 = 2410, 2685
        # x4, y4 = 2700, 2188 
        # line_eq2 = self.generate_line_equation(x3, y3, x4, y4)
        # if getch() == chr(0x1B):
        #     for servo in self.servos:
        #         self.Disable(servo)
        #     quit()
        #         ## Disable Leader
        # # time.sleep(2)
        # print("press space to disable leader2")

        # if getch() == chr(0x1B):
        #     for servo in self.servos:
        #         self.Disable(servo)
        #     quit()
                ## Disable Leader
        # time.sleep(2)
        print("press space to disable leader")

        if getch() != chr(0x20):
            # for servo in self.servos:
            #     self.Disable(servo)
            quit()
        #         ## Disable Leader
        time.sleep(5)

        for servo in self.servo_com["L2"] + self.servo_com["L1"]:
            self.Disable(servo)
        try:
            i = 0
            while i < samples:

                # # L1 -> F1
                print(self.goal)
                ###Read from the Leader
                # for servo in self.servo_com["L"]:
                #     L_present_values[servo] = self.signed(self.getj(servo),servo)  
                mx1061_present_values  = self.getj("mx1061")
                # time.sleep(0.005)                                     #l[1]
                xm540L1_present_values = self.getj("xm540L1")                                    #l[2,3,4]
                # time.sleep(0.005)                                 
                # xm430L1_present_values = self.getj("xm430L1")                                    #l[5,6,7]
                # time.sleep(0.005)                                     #l[1]

                print("xm430L1_present_values", xm430L1_present_values)
                # self.get_time(time.time(), i, "get_L")

                pro1_present_values    = mx1061_present_values + xm540L1_present_values[:2]           #f[1,2,3] <- l[1,2,3]
                # xm540F1_current_values = self.getj("xm540F1")                                    
                
                pro1_target_values    = self.rounderror(pro1_present_values, sensitivity, "pro1")                       
                xm540F1_target_values = [xm540L1_present_values[2]] #+ xm430L1_present_values[:2]    #l[4,5,6]
                # xm430F1_target_values = [int(line_eq1(xm430L1_present_values[2]))] #[int(-0.983* xm430L1_present_values[2] + 5001.863)]          #l[7]   line Mapping
                # print("xm540F1_current_values", xm540F1_current_values, "xm430F1_target_values", xm430F1_target_values ) #just print follower
                # print("========================")

                pro1_filtered_values    = self.Filter(pro1_target_values, "pro1")
                xm540F1_filtered_values =  self.Filter(xm540F1_target_values,"xm540F1")                                    #self.Filter(xm540_target_values, "xm540")
                # xm430F1_filtered_values = self.Filter(xm430F1_target_values, "xm430F1")
                # print("xm430F1_target_values", xm430F1_target_values ) 
                # print("xm430F1_filtered_values", xm430F1_filtered_values)

                #target follower
                self.goal["pro1"]    = pro1_filtered_values                      #pro_target_values
                self.goal["xm540F1"] = xm540F1_filtered_values                 #xm540_target_values
                # self.goal["xm430F1"]  = xm430F1_filtered_values

                # print("Goal", self.goal)
                ############# move code important
                self.movej(self.goal, "pro1")            
                self.movej(self.goal, "xm540F1")
                # self.movej(self.goal, "xm430F1")

                # # L2 -> F2
                # print(self.goal)
                ###Read from the Leader
                # for servo in self.servo_com["L"]:
                #     L_present_values[servo] = self.signed(self.getj(servo),servo)
                    
                mx1062_present_values  = self.getj("mx1062")                                     #l[1]
                # time.sleep(0.005) 
                xm540L2_present_values = self.getj("xm540L2")                                    #l[2,3,4]
                # time.sleep(0.005) 
                # xm430L2_present_values = self.getj("xm430L2")                                    #l[5,6,7]
                # time.sleep(0.005) 
                # print("xm430L2_present_values", xm430L2_present_values)
                # self.get_time(time.time(), i, "get_L")

                pro2_present_values    = mx1062_present_values + xm540L2_present_values[:2]        #f[1,2,3] <- l[1,2,3]
                # xm540F2_current_values = self.getj("xm540F2")                                    
                
                pro2_target_values    = self.rounderror(pro2_present_values, sensitivity, "pro2")                       
                xm540F2_target_values = [xm540L2_present_values[2]] #+ xm430L2_present_values[:2]    #l[4,5,6]
                # xm430F2_target_values = [int(line_eq2(xm430L2_present_values[2]))]                 
                # print("xm540F2_current_values", xm540F2_current_values, "xm430F2_target_values", xm430F2_target_values) #just print follower
                print("========================")

                pro2_filtered_values    = self.Filter(pro2_target_values, "pro2")
                xm540F2_filtered_values = self.Filter(xm540F2_target_values,"xm540F2")                                    #self.Filter(xm540_target_values, "xm540")
                # xm430F2_filtered_values = self.Filter(xm430F2_target_values, "xm430F2")
                # print("xm430F2_filtered_values", xm430F2_filtered_values)

                #target follower
                self.goal["pro2"]    = pro2_filtered_values                   #pro_target_values
                self.goal["xm540F2"] = xm540F2_filtered_values                 #xm540_target_values
                # self.goal["xm430F2"] = xm430F2_filtered_values

                print("Goal", self.goal)
                ############# move code important

                self.movej(self.goal, "pro2")            
                self.movej(self.goal, "xm540F2")
                # self.movej(self.goal, "xm430F2")


                #     ###combine the values into list do rounderror can change them at once
                #     if "pro" in servo:
                #         L_present[servo] = L_present_values["mx106"] + L_present_values["xm540"]
                #         ### Convert Leader angle(value) to Follower angle(value)
                #         L_present[servo] = self.rounderror(L_present[servo], 245, servo)
                #         ### Smooth the follower angles with low pass filter
                #         filtered_values[servo] = self.Filter(L_present[servo], "pro")
                #     elif servo == "xm540":
                #         L_present[servo]=[L_present_values["xm430L"][0]]
                #         filtered_values[servo] = self.Filter(L_present[servo], "xm540")
                #     elif servo == "xm430F":
                #         L_present[servo]=L_present_values["xm430L"][0:]
                #         filtered_values[servo] = self.Filter(L_present[servo], "xm430F")



                #     self.goal[servo]=filtered_values[servo]

                #     ### Move the follower to the filtered target angles 
                #     self.movej(self.goal, "pro")


                # print(
                #     "xm540:",
                #     self.value2angle(L_present, "xm540"),
                #     "  PRO:",
                #     self.value2angle(target_values, "pro"),
                # )

                if recordData != "norecord":
                    recordData.record(image_path, 0, "pro")
                else:
                    self.stats = 1

                if self.stats:
                    i += 1
                print(i)

            
            print("loop completed ")
            for servo in self.servo_com["L1"]:
                self.Enable(servo)
            for servo in self.servo_com["L2"]:
                self.Enable(servo)
            
            # print("lift for aruco record")
            # for servo in self.servo_com["F"]:
            #         print("B_hover_pos")
            #         B_hover_pos[servo] = self.signed(self.getj(servo),servo)
            #         B_hover_pos_[servo]=[-17312,75528,220859]#[B_hover_pos[servo][0], B_hover_pos[servo][1]-70000,B_hover_pos[servo][2]+40000]
            # B_graze_pos_ = dict(pro=[-17312,49371,220859])
            # B_start_pos= dict(pro=[-17312,75528,220859])
            # present_position = self.sin_movej(
            #             B_hover_pos,B_graze_pos_, 0.005, "F"
            #         )
            # present_position = self.sin_movej(
            #             B_graze_pos_,B_estart_pos, 0.005, "F"
            #         )
            # ### Capture Aruco pixel data before wiping with LF
            # #self.aruco_detector.capture_frames(image_path_side,recordData, 'end', angle=slope)  # Capture "end" frames with angle slope in degree
            # if recordData != "norecord":
            #     print("save data")
                
                
            #     recordData.save_data(csv_path, 1,1)
            #     #self.aruco_detector.save_data_to_excel(recordData,excel_path)
        except KeyboardInterrupt:
            print("Interrupted")
            for servo in self.servos:
                self.Disable(servo)
            return ""
       
        
        ### reset variables
        self.times = {}
        self.run   = [1, 1, 1, 1, 1, 1, 1,1, 1, 1, 1, 1, 1, 1]
        print("seq_num", seq_num,
            "---------------------------------------------------------",
        )

    def Automation_data(self, seq_num, recordData, samples, cycles, trails, Wipe_trj):
        # self.seq_num=seq_num
        for servo in self.servos:
            self.Enable(servo)
        self.stats = 1
        try:
            goal = self.goal
            print(goal)  # [132474,48933,-149]
            for servo in self.servos:
                if "pro" in servo:
                    # ,[132474,48933,-149],[164043,62878,17687],[246194,41472,44186],[140788,67811,50354],[159661,64144,-38188],[250000,26955,-1080]]
                    for ii, pro_ang in enumerate([Wipe_trj]):  # [[242300,17500,0]]:
                        print(ii, pro_ang)
                        for trail in range(trails):
                            if recordData != "norecord":
                                # Create Directory
                                (
                                    csv_path,
                                    image_path,
                                    image_path_side,
                                ) = recordData.create_datadir(seq_num)
                            pro_current_values = self.getj("pro")
                            initial_pro_angles = self.value2angle(
                                pro_current_values, servo
                            )
                            pro_current_values = self.signed(pro_current_values, servo)
                            print("present_pro_angles", initial_pro_angles)
                            run = 1
                            # sin_step=0.008
                            present_position = self.sin_record_wipe(
                                pro_current_values,
                                pro_ang,
                                samples,
                                cycles,
                                recordData,
                                image_path,
                                image_path_side,
                                servo,
                            )
                            if recordData != "norecord":
                                recordData.save_data(csv_path)
                            # self.get_seq(time.time(), str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            print("trail completed ", trail)
                            seq_num += 1
                            print("Draw line")
                            if getch() != chr(0x20):
                                break
                        print("Change angle", ii)
                        if getch() != chr(0x20):
                            break

        except KeyboardInterrupt:
            print("Interrupted")
            for servo in self.servos:
                self.Disable(servo)

        return seq_num

    def Automation_data_C(self, seq_num, recordData, samples, cycles, trails, Wipe_trj, B_hover_pos, comments = 0,):
        for servo in self.servos:
            self.Enable(servo)
        self.stats = 1
        try:
            goal = self.goal
            print(goal)  # [132474,48933,-149]
            for servo in self.servos:
                if "pro" in servo:
                    # ,[132474,48933,-149],[164043,62878,17687],[246194,41472,44186],[140788,67811,50354],[159661,64144,-38188],[250000,26955,-1080]]
                    for ii, pro_ang in enumerate([Wipe_trj]):  # [[242300,17500,0]]:
                        print(ii, pro_ang)
                        B_lowpos_nt = pro_ang[0]
                        for trail in range(trails):
                            if recordData != "norecord":
                                # Create Directory
                                (
                                    csv_path,
                                    image_path,
                                    image_path_side,
                                ) = recordData.create_datadir(seq_num)
                            else:
                                image_path, image_path_side = 0, 0

                            run = 1
                            # sin_step=0.008
                            pro_current_values = self.getj("pro")
                            initial_pro_angles = self.value2angle(
                                pro_current_values, "pro"
                            )
                            print("present_pro_angles", trail, initial_pro_angles)

                            # move to get correct initial current values
                            # # Wipe_init = [pro_current_values,pro_ang[0]]
                            # # present_position=self.sin_movej(Wipe_init[0],Wipe_init[1],0.02,"pro")
                            Wipe_init = [
                                pro_current_values,
                                [
                                    B_lowpos_nt[0],
                                    B_lowpos_nt[1] - 20000,
                                    B_lowpos_nt[2],
                                ],
                                B_lowpos_nt,
                            ]
                            # Wipe_init = [pro_current_values,[yposition[0], 64369, yposition[2]],yposition[:3]]

                            # present_position=DymControl.sin_movej(Wipe_init[0],Wipe_init[1],020.,"pro")
                            for w in range(len(Wipe_init) - 1):
                                print(Wipe_init[w], Wipe_init[w + 1])
                                present_position = self.sin_movej(
                                    Wipe_init[w], Wipe_init[w + 1], 0.01, servo
                                )

                            # Record the wipe
                            present_position = self.sin_record_wipe(
                                pro_ang[0],
                                pro_ang[1:],
                                samples,
                                cycles,
                                recordData,
                                image_path,
                                image_path_side,
                                B_hover_pos,
                                servo,
                            )
                            if recordData != "norecord":
                                recordData.save_data(csv_path, comments, trail)
                            # self.get_seq(time.time(), str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            print("trail completed ", trail)
                            seq_num += 1
                            # print("Draw line")
                            # if getch() != chr(0x20):
                            #     break
                        # print("Change angle", ii)
                        # if getch() != chr(0x20):
                        #     break

        except KeyboardInterrupt:
            print("Interrupted")
            for servo in self.servos:
                self.Disable(servo)

        return seq_num, csv_path

        # Start Data Collection
