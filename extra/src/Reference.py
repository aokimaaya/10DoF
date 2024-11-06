#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.

#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# *******************************************************************************


# *******************************************************************************
# ***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code.
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import itertools
import json
import os
import random
import threading
import time

import numpy

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


from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
# ***** (Use only one definition at a time) *****
# MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
MY_DXL = "P_SERIES"  # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


##Ganga code##=======================================
#
# A1=30
# A2=0
# FREQ=0.1
# Dur=15
# stt=0.015
# AF = 4294967295
# W =1.5
# v=2
vel = ""
A1 = 30
A2 = 180
FREQ = 0.1
Dur = 10
stt = 0.2
AF = 4294967295
W = 0
v = 8
# angle range mapping
angle_list1 = range(-180, 181)
theta_ranges1 = [round(n * 2788.461111111) for n in range(-180, 181)]

d1 = dict(zip(angle_list1, theta_ranges1))
angle_list2 = range(-180, 181)
theta_ranges2 = [round(n * 11.375) for n in range(-180, 181)]

d2 = dict(zip(angle_list2, theta_ranges2))
j = 1
# angle_list2 = range(0,361)
# theta_ranges2 = [round(n*11.375) for n in range(4097)]
DXL_MOVING_STATUS_THRESHOLD = 2000 * j * 2
DXL_MOVING_STATUS_THRESHOLD2 = 30 * j
DXL_MOVING_STATUS_THRESHOLD3 = 2000 * j
DXL_MOVING_unit = 1000 * j * 2
DXL_MOVING_unitx2 = 15 * j
DXL_MOVING_unitx3 = 1000 * j
div = 1
# d2 = dict(zip(angle_list2, theta_ranges2))
vel_list = range(-1023, 1023)

# State position
State_ = {
    "A": "Angle Position Control",
    "S": "Sweep Action",
    "K": "Arrow control",
    "I": "Intial Neutral Position",
    "W": "Wiping Action",
    "SI": "Sinusoidal wiping Action",
    "SM": "Space Mouse Control",
    "VM": "Velocity Spm Control",
    "LFP": "Master Slave Position Control",
}

# import signal
import time

# class GracefulKiller:
#   kill_now = False
#   def __init__(self):
#     signal.signal(signal.SIGINT, self.exit_gracefully)
#     signal.signal(signal.SIGTERM, self.exit_gracefully)
#
#   def exit_gracefully(self, *args):
#     self.kill_now = True


def twos_comp(val, bits=16):
    """compute the 2's complement of int value"""
    if bits == 0:  # Use as many bits needed for the value.
        bits = val.bit_length()
    return ((val & (2**bits) - 1) - (2**bits)) * -1


# import signal
import matplotlib.pyplot as plot
import numpy as np


def Plot(Ang_, N=180, i=0):
    # Get x values of the sine wave
    if i == 0:

        time = [x[0] for x in Ang_]
        time1 = [x[0] for x in PR]

        # Amplitude of the sine wave is sine of a variable like time
        amplitude = [-(d[N] - x[1]) for x in Ang_]
        z = [-(d[N] - x[1]) for x in PR]
    else:
        time = [x[0] for x in Ang_]
        # Amplitude of the sine wave is sine of a variable like time
        amplitude = [-(N - x[1 + i]) for x in Ang_]
        z = [-(N - x[3 + i]) for x in Ang_]
    # Plot aN sine wave using time and amplitude obtained for the sine wave
    plot.plot(time, amplitude, color="r", label="Goalpos")
    plot.plot(time1, z, color="g", label="Actual Pos")
    # Give a title for the sine wave plot
    plot.title("Angle comparision")
    # Give x axis label for the sine wave plot
    plot.xlabel("Time")
    # Give y axis label for the sine wave plot
    plot.ylabel("Amplitude = sin(time)")
    plot.grid(True, which="both")
    plot.axhline(y=0, color="k")
    plot.legend()
    plot.show()


previous_time = time.time()
amount_step = 0
global Ang_, Ang, PR
global key1, key2, key3
key1 = 0
key2 = 180
key3 = 50
Amp = 45
# Ang_=[]
PR = []


def toSigned32(n):
    n = n & 0xFFFFFFFF
    return n | (-(n & 0x80000000))


def pos2ang(pos):
    snval = toSigned32(pos)
    return round((snval / 501923) * 180)


pos_ll = []


def moveup():
    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
    )

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    # p2=d1[int(float(key1))]
    # if servo_ang(p1,p2):
    # print("p<t")
    if p1 < 501923:
        # print("p<t")
        goal = p1 + DXL_MOVING_unit
    else:
        goal = p1 - AF + DXL_MOVING_unit
    # else:
    #     # print("p>t")
    #     if (p1<501923 ):
    #         goal = p1 - DXL_MOVING_unit
    #     else:
    #         goal= p1 -AF - DXL_MOVING_unit
    # if goal%10==0:
    #     print("PROup",goal,p1)
    dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
        portHandler, DXL_ID1, ADDR_GOAL_POSITION1, goal
    )  # d[int(float(key))])# d[d[int(float(key))]])

    return goal
    # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,goal)
    # pass


def movedown():
    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
    )

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    sp = toSigned32(p1)
    # p2=d1[int(float(p1))]
    # if servo_ang(p1,p2):
    # print("p<t")
    # if (p1<501923 ):
    #     # print("p<t")
    #     goal = p1 + DXL_MOVING_unit
    # else:
    #     goal= p1 -AF + DXL_MOVING_unit
    # else:
    #     # print("p>t")
    if p1 < 501923:
        goal = p1 - DXL_MOVING_unit
    else:
        goal = p1 - AF - DXL_MOVING_unit
    # if goal%10==0:
    #     # print("PROup",goal,p1)
    dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
        portHandler, DXL_ID1, ADDR_GOAL_POSITION1, goal
    )  # d[int(float(key))])# d[d[int(float(key))]])

    return goal


def moveprop(key1, ID):
    # print("moveprov")
    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        portHandler, ID, ADDR_PRESENT_POSITION1
    )
    # dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, DXL_ID1, 20)

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    # dxl_present_velocity1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, ID, ADDR_PRESENT_VELOCITY1)
    sp = toSigned32(p1)
    p = pos2ang(p1)
    print("moveprov----", "p1", p1, "p", p, "sp", sp)
    # print("-----value  ",p1,"  signed  ",sp,"  Angle  " , p)
    # p2=d1[int(float(p1))]
    # if servo_ang(p1,p2):
    # print("p<t")
    # if (p>90):
    #     # print("p<t")
    #     goalx = p #+ DXL_MOVING_unit
    # else:
    #     goalx= p1 - AF #+ DXL_MOVING_unit

    # print("goalx",goalx,p1)
    # else:
    #     # print("p>t")
    goal = key1
    # if(p>88):
    #     goal = -200
    #     # print("-50")
    # elif (p>80):
    #     goal = key1/(2**(p-80))
    #     # print(p,goal)
    # elif(p< - 88):
    #     goal = 200
    #     # print("50")
    # elif(p<-80):
    #     goal= key1/(2**(-80-p))
    #
    # if key1 == 0:
    #     goal = 0
    # print(p1,goal)
    # if ((p1-A)<-223077):
    #     goal = key1/2
    #     print(p1-A,goal)
    # elif(p1>223077):
    #     goal= key1/2
    #     print(p1,goal)
    # goal= key1
    # print("goal",dxl_present_velocity1,goal,sp)
    # if goal%10==0:
    #     # print("PROup",goal,p1)
    dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
        portHandler, ID, ADDR_GOAL_POSITION1, int(goal)
    )  # d[int(float(key))])# d[d[int(float(key))]])
    # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(Handler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(goal))# d[int(float(key))])# d[d[int(float(key))]])

    return p1


def moveprov(key1):
    # print("moveprov")
    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_POSITION1
    )
    # dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(Handler, DXL_ID1, 20)

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    dxl_present_velocity1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_VELOCITY1
    )
    sp = toSigned32(p1)
    p = pos2ang(p1)
    # print("moveprov----","p1",p1,"p",p,"sp",sp)

    # p2=d1[int(float(p1))]
    # if servo_ang(p1,p2):
    # print("p<t")
    # if (p>90):
    #     # print("p<t")
    #     goalx = p #+ DXL_MOVING_unit
    # else:
    #     goalx= p1 - AF #+ DXL_MOVING_unit

    # print("goalx",goalx,p1)
    # else:
    #     # print("p>t")
    goal = key1
    # if(p>88):
    #     goal = -200
    #     # print("-50")
    # elif (p>80):
    #     goal = key1/(2**(p-80))
    #     # print(p,goal)
    # elif(p< - 88):
    #     goal = 200
    #     # print("50")
    # elif(p<-80):
    #     goal= key1/(2**(-80-p))
    # #
    # if key1 == 0:
    #     goal = 0
    # print(p1,goal)
    # if ((p1-A)<-223077):
    #     goal = key1/2
    #     print(p1-A,goal)
    # elif(p1>223077):
    #     goal= key1/2
    #     print(p1,goal)
    # goal= key1
    # print("goal",dxl_present_velocity1,goal,sp)
    # if goal%10==0:
    #     # print("PROup",goal,p1)
    dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
        Handler, DXL_ID1, ADDR_GOAL_VELOCITY1, int(goal)
    )
    # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(Handler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(goal))# d[int(float(key))])# d[d[int(float(key))]])

    return p1


def moveprov3(key3):
    # print("moveprov")
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_POSITION1
    )
    # dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(Handler, DXL_ID1, 20)

    p1 = dxl_present_position3  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    dxl_present_velocity3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_VELOCITY1
    )
    sp = toSigned32(p1)
    p = pos2ang(p1)
    # print("moveprov----","p1",p1,"p",p,"sp",sp)

    # p2=d1[int(float(p1))]
    # if servo_ang(p1,p2):
    # print("p<t")
    # if (p>90):
    #     # print("p<t")
    #     goalx = p #+ DXL_MOVING_unit
    # else:
    #     goalx= p1 - AF #+ DXL_MOVING_unit

    # print("goalx",goalx,p1)
    # else:
    #     # print("p>t")
    goal = key3
    # if(p>88):
    #     goal = -200
    #     # print("-50")
    # elif (p>80):
    #     goal = key3/(2**(p-80))
    #     # print(p,goal)
    # elif(p< - 88):
    #     goal = 200
    #     # print("50")
    # elif(p<-80):
    #     goal= key3/(2**(-80-p))
    # if key3 == 0:
    #     goal = 0
    # print(p1,goal)
    # if ((p1-A)<-223077):
    #     goal = key1/2
    #     print(p1-A,goal)
    # elif(p1>223077):
    #     goal= key1/2
    #     print(p1,goal)
    # goal= key1
    # print("goal",dxl_present_velocity1,goal,sp)
    # if goal%10==0:
    #     # print("PROup",goal,p1)
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        Handler, DXL_ID3, ADDR_GOAL_VELOCITY1, int(goal)
    )
    # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(Handler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(goal))# d[int(float(key))])# d[d[int(float(key))]])

    return p1


def movexm540v(key2):
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )

    p = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goal = key2  # b1 - DXL_MOVING_unitx2
    # if (p<910):
    #     goal = 5
    #
    # elif(p>910 and p<1000):
    #     goal = key2/(2**(p-910))
    #     # print(p,goal)
    #     # print("forward")
    # elif (p>=1000 and p < 2958):
    #     goal = key2
    #     # print("p")
    # elif(p > 2958 and p < 3071):
    #     goal = key2/(2**(-3071-p))
    #     # print("50")
    # elif(p>3071):
    #     goal = -5
    # else:
    #     goal=0
    # print("movexm540 -----", goal)
    if key2 == 0:
        goal = 0
    # elif(p>=3071):
    #     goal= key2/(2**(-3071-p))
    #     print(p,goal)
    # else:
    print("goalxm540", p, goal)
    # print("goal2",goalx,d2[int(float(b1))],b1)
    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        Handler, DXL_ID2, ADDR_GOAL_VELOCITY2, int(goal)
    )

    return p  # Ang[amount_step]


# def moveleftV3(vel):
#     #velocity control for 3
#     dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler.read4ByteTxRx(Handler, DXL_ID, ADDR_PRESENT_VELOCITY)
#
#     b1=dxl_present_position3#d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
#
#     goalx = b1 + vel#DXL_MOVING_STATUS_THRESHOLD
#     print(goalx,dxl_comm_result3, dxl_error3)
#     # if goalx%10==0:
#     #     print("xm540left",goalx,b1)
#     # dxl_comm_result3,dxl_present_position3, dxl_error3 = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_PRESENT_VELOCITY,goalx)
#     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_GOAL_VELOCITY, int(vel))
#
#
#     return goalx
#
# def moverightV3(vel):
#     #velocity control for 3
#     dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler.read4ByteTxRx(Handler, DXL_ID, ADDR_PRESENT_VELOCITY)
#
#     b1=dxl_present_position3#d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
#
#     goalx = b1 + vel#DXL_MOVING_STATUS_THRESHOLD
#     print(goalx,dxl_present_position3,dxl_comm_result3, dxl_error3)
#     # if goalx%10==0:
#     #     print("xm540right",goalx,b1)
#     # dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_PRESENT_VELOCITY,goalx)
#     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_GOAL_VELOCITY, int(vel))
#
#
#     return goalx
def moveleftV3(vel):
    print("velocity control for 3")
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_VELOCITY3
    )

    b1 = dxl_present_position3  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    # goalx = b1 + DXL_MOVING_unitx3
    # print(goalx,dxl_comm_result3, dxl_error3)

    print(vel)
    # if goalx%10==0:
    #     print("xm540left",goalx,b1)
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        Handler, DXL_ID3, ADDR_GOAL_VELOCITY3, int(vel)
    )

    return goalx


def moverightV3(vel):
    # velocity control for 3
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_VELOCITY3
    )

    b1 = dxl_present_position3  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    # goalx = b1 - DXL_MOVING_unitx3
    # print(goalx,dxl_comm_result3, dxl_error3)
    print(vel)
    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        Handler, DXL_ID3, ADDR_GOAL_VELOCITY3, int(vel)
    )

    return vel


def moveleft2():
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position3  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = b1 + DXL_MOVING_unitx3

    # if goalx%10==0:
    #     print("xm540left",goalx,b1)
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        Handler, DXL_ID3, ADDR_GOAL_POSITION2, goalx
    )

    return goalx


def moveright2():
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position3  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = b1 - DXL_MOVING_unitx3

    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        Handler, DXL_ID3, ADDR_GOAL_POSITION2, goalx
    )

    return goalx


def moveleft():
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = b1 + DXL_MOVING_unitx2

    # if goalx%10==0:
    #     print("xm540left",goalx,b1)
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
    )

    return goalx


def moveright():
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = b1 - DXL_MOVING_unitx2

    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
    )

    return goalx


def movexm540(key2):
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = d2[int(float(key2))]  # b1 - DXL_MOVING_unitx2

    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
    )

    return goalx  # Ang[amount_step]


def movexm540sin(cc):
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]

    goalx = Ang[cc]  # d2[int(float(key2))]#b1 - DXL_MOVING_unitx2

    # if goalx%10==0:
    #     print("xm540right",goalx,b1)
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
    )

    return goalx  #


# def read_angles(stt):
#     with open('listfile_'+str(stt)+'_'+str(A1)+'_'+str(FREQ)+'.txt','r') as filehandle:
#     #'listfile_'+str(stt)+'_'+str(FREQ)+'pro.txt', 'r') as filehandle:
#         Ang_ = json.load(filehandle)
#     return Ang_


def get_seq(PR, stt):
    # data_lines = [ for seq_id in Ang_]
    # open output file for writing
    with open(
        "Plotangles_e_"
        + str(DXL_MOVING_STATUS_THRESHOLD2)
        + "_"
        + str(DXL_MOVING_STATUS_THRESHOLD)
        + ".txt",
        "w",
    ) as filehandle:
        json.dump(PR, filehandle)


def a2(d, dxl):
    prepos = list(d.keys())[
        list(d.values()).index(min(d.values(), key=lambda x: abs(x - dxl)))
    ]
    return prepos


def servo_ang(present, target):
    # print("present,target",present,target)
    if present < 501923:
        if present <= target:
            return 1
        else:
            return 0
    else:
        if present - AF <= target:
            return 1
        else:
            return 0


def servo_delay(p1, p2, b1, b2):
    # AF=4294967295
    td2 = stt * abs(b2 - b1) / DXL_MOVING_unitx2
    if p1 < 501923:
        td1 = stt * abs(p2 - p1) / DXL_MOVING_unit
    else:
        td1 = stt * abs(p2 - p1 + AF) / DXL_MOVING_unit
    return max(td1, td2 / div) + 2


def wipexm540():
    print(State_[St])
    A1 = 30
    A2 = 180
    FREQ = 0.1
    Dur = 10
    stt = 0.03
    print(
        """            Start wiping
    Amplitude=:%03d
    Neutral Angle=%03d
    FREQ in Hz=%1.2f
    Duration in ms=%03d
    Time step in ms=%1.4f """
        % (A1, A2, FREQ, Dur, stt)
    )

    # while 1:
    #     A1 = input("Amplitude (45) ")
    #     A1=45
    #     if A1 != "":
    #         break
    # while 1:
    #     A2 = input("Neutral Angle (180)")
    #     A2=180
    #     if A2 != "":
    #         break
    # # if
    # while 1:
    #     FREQ = input("Frequency in Hz (0.5)")
    #     FREQ=0.5
    #     if FREQ != "":
    #         break
    # while 1:
    #     Dur = input("Time Duration in Seconds (10)")
    #     Dur=10
    #     if Dur != "":
    #         break
    # while 1:
    #     stt = input("Step amount (0.01)")
    #     stt=0.01
    #     if stt != "":
    #         break
    # while 1:
    #     print("Enter to start Sweep with Neutral Angle:%03d and Amplitude:%03d",A2,A1)
    #     if getch() == chr(0x1b):
    #         break

    # print("inside")
    # FREQ = 0.5

    t0 = time.time()
    # while True:
    #     if int(A1)>int(A2):
    #         print("Please give the angle in range with neutral angle")
    #         break
    #     t = time.time()
    #
    #     if (t - t0) > int(Dur)+1:
    #         print("Time Taken%8.8f" %(t-t0))
    #         break
    #     sinv=d[int(A1)] * numpy.sin(2 * numpy.pi * float(FREQ) * t)
    #     key =d[int(A2)] + sinv
    #     aa=min(d.values(), key=lambda x:abs(x-key))
    #     an=list(d.keys())[list(d.values()).index(aa)]
    #     # print(time.time(),pos,list(d.keys())[list(d.values()).index(aa)])
    #     # key =d[A2] + d[int(A1)] * numpy.sin(2 * numpy.pi * FREQ * t))
    #     print(time.time()-t0,key)
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(Handler, DXL_ID, ADDR_PRESENT_POSITION)
    #     # print("S=[ID:%8.8f] GoalPos:%03d PresentPos:%03d" % (time.time()-t0, key, dxl_present_position))
    #     prepos=list(d.keys())[list(d.values()).index(min(d.values(), key=lambda x:abs(x-dxl_present_position)))]
    #     Ang_.append((time.time()-t0, round(key),an))#, dxl_present_position,prepos))
    #
    #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_GOAL_POSITION, round(key))
    #     while abs(key- dxl_present_position) < 5:
    #         print("diff%03d"% (key - dxl_present_position))
    #         break
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     print("[ID:%03d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID, d[int(key)], int(key) , dxl_present_position))
    #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(Handler, DXL_ID, ADDR_GOAL_POSITION, d[int(key)])
    #     time.sleep(0.0005)
    Ang_ = read_angles(stt)
    Ang = [round(x[1]) for x in Ang_]
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        Handler, DXL_ID, ADDR_GOAL_POSITION, round(Ang[0])
    )
    print("Set angle to start Pos", round(Ang[0]))
    time.sleep(3)
    Angt = [x[0] for x in Ang_]
    print("Before")
    t0 = time.time()
    # signal.signal(signal.SIGALRM, sendang)
    print("375")

    # signal.setitimer(signal.ITIMER_REAL, stt, stt)

    time.sleep(25)
    print("Wipe end")


# from keyboard im Key, Listener
keys = ["down", "up", "left", "right", "shift_r", "page_up", "page_down", "space"]
found = {}
cc = 0


def mapspace(spm):
    if spm[2] > 180 and spm[3] < -180:
        # key1=moveup()
        # key2=moveleft()
        print("up,left")
    elif spm[2] > 180 and spm[3] < 180:
        # key1=moveup()
        # key2=moveright()
        print("up,right")
    elif spm[2] < -180 and spm[3] < 180:
        # key1=movedown()
        # key2=moveright()
        print("down,right")
    elif spm[2] < -180 and spm[3] < -180:
        # key1=movedown()
        # key2=moveleft()
        print("down,left")
    elif spm[2] > 180:  # found["up"]:
        # key1=moveup()
        print("up")
    elif spm[2] < -180:  # found["down"]:
        # key1=movedown()
        print("down")
    elif spm[3] < 180:  # found["right"]:
        # key2=moveright()
        print(key)
    elif spm[3] < -180:  # found["left"]:

        # key2=moveleft()
        print("left")


def on_press(key):
    # print('{0} pressed'.format(
    #     key))
    global cc
    for k in keys:

        if Key[k] == key:
            # print(f"{k} was pressed")
            found[k] = True
        # print(found)
    if found["up"] and found["left"]:
        key1 = moveup()
        key2 = moveleft()
        print("up,left")
    elif found["up"] and found["right"]:
        key1 = moveup()
        key2 = moveright()
        print("up,right")
    elif found["down"] and found["right"]:
        key1 = movedown()
        key2 = moveright()
        print("down,right")
    elif found["down"] and found["left"]:
        key1 = movedown()
        key2 = moveleft()
        print("down,left")
    elif found["up"]:
        key1 = moveup()
        print("up")
    elif found["down"]:
        key1 = movedown()
        print("down")
    elif found["right"]:
        key2 = moveright()
        print(key)
    elif found["left"]:

        key2 = moveleft()
        print("left")
    elif found["shift_r"]:
        cc = 0
        key2 = movexm540(180)
        print("shift_r")
    elif found["page_up"]:
        key2 = movexm540(190)
        print("page_up")
    elif found["page_down"]:
        key2 = movexm540(170)
        print("page_down")  # movexm540sin()
    elif found["space"]:
        if cc < len(Ang_) - 1:
            cc = cc + 1
        else:
            cc = 0
        print(cc)
        key2 = movexm540sin(cc)
        print("wipesin")  # movexm540sin()


def on_release(key):
    # print('{0} release'.format(
    #     key))
    for x in keys:

        found[x] = False
    if key == Key.end:
        # Stop listener
        return False


# Collect events until released


def sendang(arg1, args2):
    xm540, PRO = 0, 0

    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_POSITION1
    )

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    p2 = d1[int(float(key1))]

    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )
    if abs(d2[int(key2)] - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD2:

        xm540 = 1

    if p1 < 501923:
        if abs(p2 - p1) < DXL_MOVING_STATUS_THRESHOLD:
            PRO = 1

    else:
        if abs(p2 - p1 + AF) < DXL_MOVING_STATUS_THRESHOLD:
            PRO = 1

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    b2 = d2[int(float(key2))]
    global previous_time
    global amount_step
    global goal
    global goalx
    current_time = time.time()
    # --------------------------
    if not PRO:
        if servo_ang(p1, p2):
            # print("p<t")
            if p1 < 501923:
                # print("p<t")
                goal = p1 + DXL_MOVING_unit
            else:
                goal = p1 - AF + DXL_MOVING_unit
        else:
            # print("p>t")
            if p1 < 501923:
                goal = p1 - DXL_MOVING_unit
            else:
                goal = p1 - AF - DXL_MOVING_unit
        if goal % 10 == 0:
            print("PRO", current_time, goal, p2, p1)
        pp = goal
        dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
            Handler, DXL_ID1, ADDR_GOAL_POSITION1, goal
        )
        dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
            Handler, DXL_ID3, ADDR_GOAL_POSITION1, goal
        )

    # ----------------------------------

    previous_time = current_time

    # print("xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID2, d2[int(key2)], int(key2) , dxl_present_position2))
    # if amount_step==0:
    if not xm540 and amount_step % div == 0:

        if servo_ang(b1, b2):
            goalx = b1 + DXL_MOVING_unitx2
        else:
            goalx = b1 - DXL_MOVING_unitx2

        if goalx % 10 == 0:
            print("xm540=", current_time, goalx, b2, b1)
        bb = goalx
        dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
            Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
        )
        # delay(1)
        # dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(Handler, DXL_ID3, ADDR_GOAL_POSITION2,goalx)

    if abs(d2[int(key2)] - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD2:

        xm540 = 1

    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error1))

    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error2))

    # print(xm540,PRO)
    if xm540 & PRO:
        print("stop!!in")
        # signal.setitimer(#signal.ITIMER_REAL, 0.0, 0.0)
        # #signal.#signal(signal.SIGALRM, stop)
        print("stop!!")
    amount_step = amount_step + 1

    # St=""


count = 0
# return p1,b1
def sendangwipe(arg1, args2):
    global previous_time
    global amount_step
    global goal
    global goalx
    global count
    xm540, PRO = 0, 0

    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_POSITION1
    )

    p1 = dxl_present_position1  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    p2 = d1[int(float(key1))]

    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )
    # if abs(d2[int(key2)] - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD2 :
    #
    #     xm540 =1

    if amount_step > len(Ang_) - 1:
        xm540 = 1
    if p1 < 501923:
        if abs(p2 - p1) < DXL_MOVING_STATUS_THRESHOLD:
            PRO = 1

    else:
        if abs(p2 - p1 + AF) < DXL_MOVING_STATUS_THRESHOLD:
            PRO = 1

    b1 = dxl_present_position2  # d1[int(float(A1))] #list(d1.keys())[list(d.values()).index(A1)]
    b2 = d2[int(float(key2))]

    current_time = time.time()
    # --------------------------
    if not PRO and count % 2 == 0:
        if servo_ang(p1, p2):
            # print("p<t")
            if p1 < 501923:
                # print("p<t")
                goal = p1 + DXL_MOVING_unit
            else:
                goal = p1 - AF + DXL_MOVING_unit
        else:
            # print("p>t")
            if p1 < 501923:
                goal = p1 - DXL_MOVING_unit
            else:
                goal = p1 - AF - DXL_MOVING_unit
        if goal % 10 == 0:
            print("PRO", current_time, goal, p2, p1)
        pp = goal
        dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
            Handler, DXL_ID1, ADDR_GOAL_POSITION1, goal
        )
    # ----------------------------------

    previous_time = current_time

    # print("xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID2, d2[int(key2)], int(key2) , dxl_present_position2))
    # if amount_step==0:
    if not xm540 and amount_step < len(Ang_) - 1:

        # if servo_ang(b1,b2):
        #     goalx = b1 + d2[30] * numpy.sin(2 * numpy.pi * FREQ * amount_step*stt)
        # else:
        #     goalx = b1 - d2[30] * numpy.sin(2 * numpy.pi * FREQ * amount_step*stt)Ang[amount_step]
        # goalx = b2 + d2[30] * numpy.sin(2 * numpy.pi * FREQ * amount_step*stt)
        goalx = Ang[amount_step]
        # if goalx%10==0:
        print("xm540=", current_time, goalx, b2, b1)
        goalx = round(goalx)
        dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
            Handler, DXL_ID2, ADDR_GOAL_POSITION2, goalx
        )
        # aa=min(d2.values(), key=lambda x:abs(x-goalx))
        # pp=list(d2.keys())[list(d2.values()).index(aa)]
        # print( amount_step*stt,pp,goalx)

        # if (2 * numpy.pi * FREQ * amount_step*stt) % (2*numpy.pi / FREQ) == 0 :
        print(Ang_[amount_step], amount_step)

    # amount_step=0

    # count= count+1
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!count",count,amount_step)

    # print(cc)
    # key2=movexm540sin(cc)
    # print("wipesin")#movexm540sin()

    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error1))

    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
    elif dxl_error1 != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error2))

    print(xm540, PRO)
    if xm540 & PRO:
        print("stop!!in")
        # signal.setitimer(#signal.ITIMER_REAL, 0.0, 0.0)
        # signal.signal(signal.SIGALRM, stop)
        print("stop!!")
    amount_step = amount_step + 1

    # St=""

    return p1, b1


def wipe(key1, key2, key3):
    t0 = time.time()

    # while 1:
    #         key1 = input("Input Goal Angle  for PROs ")
    #         if key1 != "":
    #             break
    # while 1:
    #         key2 = input("Input Goal Angle  for xm540s ")
    #         if key2 != "":
    #             break

    print("Before")
    t0 = time.time()

    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_POSITION1
    )
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )
    Dur = servo_delay(
        dxl_present_position1, d1[int(key1)], dxl_present_position2, d2[int(key2)]
    )
    print("estimated", Dur)
    time.sleep(2)
    dxl_present_position3, dxl_comm_result3, dxl_error3 = packetHandler3.read4ByteTxRx(
        Handler, DXL_ID3, ADDR_PRESENT_POSITION1
    )

    # signal.signal(signal.SIGALRM, sendang)
    print("375")

    # signal.setitimer(signal.ITIMER_REAL, stt, stt)

    time.sleep(5 + int(Dur))

    print("382")

    t1 = time.time()
    print("t1-t0", t1 - t0)
    # return key1,key2


def wipe_sin(key1, key2, Amp):
    t0 = time.time()

    # while 1:
    #         key1 = input("Input Goal Angle  for PROs ")
    #         if key1 != "":
    #             break
    # while 1:
    #         key2 = input("Input Goal Angle  for xm540s ")
    #         if key2 != "":
    #             break

    print("Before")
    t0 = time.time()

    dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
        Handler, DXL_ID1, ADDR_PRESENT_POSITION1
    )
    dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(
        Handler, DXL_ID2, ADDR_PRESENT_POSITION2
    )
    Dur = servo_delay(
        dxl_present_position1, d1[int(key1)], dxl_present_position2, d2[int(key2)]
    )
    print("estimated", Dur)
    time.sleep(2)

    # signal.signal(signal.SIGALRM, sendangwipe)
    print("375")

    # signal.setitimer(signal.ITIMER_REAL, stt, stt)

    time.sleep(5 + int(Dur * 6))

    print("382")

    t1 = time.time()
    print("t1-t0", t1 - t0)


def stop(arg1, args2):
    start_time = time.time()
    set_thread = threading.Thread(target=get_seq(PR, stt))
    set_thread.start()

    # print('Execution time for JSON dump: %s seconds' % (time.time() - start_time))
    print("stop!!in")
    # killer = GracefulKiller()
    #   while not killer.kill_now:
    #     time.sleep(1)
    #     print("doing something in a loop ...")
    # print(',\n'.join(str(x) for x in PR))

    # Plot(Ang_,A2,i=0)
    # signal.setitimer(signal.ITIMER_REAL,0.0, 0)
    # signal.setitimer( signal.ITIMER_REAL, seconds  = 0,interval = 0.0)
    # print('\n'.join(str(x for x in PR)))
    # exit()


##Ganga code end##=============================================
# Control table address
if MY_DXL == "X_SERIES" or MY_DXL == "MX_SERIES":
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    DXL_MINIMUM_POSITION_VALU2 = (
        0  # Refer to the Minimum Position Limit of product eManual
    )
    DXL_MAXIMUM_POSITION_VALUE = (
        359  # Refer to the Maximum Position Limit of product eManual
    )
    BAUDRATE = 57600
elif MY_DXL == "PRO_SERIES":
    ADDR_TORQUE_ENABLE = 562  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION = 596
    ADDR_PRESENT_POSITION = 611
    DXL_MINIMUM_POSITION_VALUE = (
        -150000
    )  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = (
        150000  # Refer to the Maximum Position Limit of product eManual
    )
    BAUDRATE = 57600
elif MY_DXL == "P_SERIES" or MY_DXL == "PRO_A_SERIES":
    ADDR_TORQUE_ENABLE1 = 512  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION1 = 564
    ADDR_PRESENT_POSITION1 = 580
    DXL_MINIMUM_POSITION_VALUE1 = (
        -501923
    )  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE1 = (
        501923  # Refer to the Maximum Position Limit of product eManual
    )
    # DXL_MINIMUM_POSITION_VALUE1  =-88 # Refer to the Minimum Position Limit of product eManual
    # DXL_MAXIMUM_POSITION_VALUE1  = 88  # R
    BAUDRATE = 115200  # 1000000#115200  #57600#1000000# 57600
    ADDR_Present_Current = 574
    ADDR_Current_Limit = 38
    ADDR_HARDWARE_ERROR = 518

    ADDR_TORQUE_ENABLE2 = 64
    ADDR_GOAL_POSITION2 = 116
    ADDR_PRESENT_POSITION2 = 132
    DXL_MINIMUM_POSITION_VALUE2 = (
        0  # Refer to the Minimum Position Limit of product eManual
    )
    DXL_MAXIMUM_POSITION_VALUE2 = (
        359  # Refer to the Maximum Position Limit of product eManual
    )
    DXL_MINIMUM_VELOCITY_VALUE3 = (
        -1023
    )  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_VELOCITY_VALUE3 = 1023
    # BAUDRATE                    = 57600
    # 1
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_VELOCITY1 = 552
    ADDR_PRESENT_VELOCITY1 = 576
    #
    ADDR_OPERATING_MODE3 = 11
    ADDR_GOAL_VELOCITY3 = 552
    ADDR_PRESENT_VELOCITY3 = 576
    # BAUDRATE                    = 57600
    ADDR_Profile_Velocity = 560
    ADDR_Profile_Acceleration = 556
    ADDR_Profile_Velocity2 = 112
    ADDR_Profile_Acceleration2 = 108
    ADDR_OPERATING_MODE2 = 11
    ADDR_GOAL_VELOCITY2 = 104
    ADDR_PRESENT_VELOCITY2 = 128
    ##Master
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_VELOCITY1M = 104
    ADDR_PRESENT_VELOCITY1M = 128
elif MY_DXL == "XL320":
    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_PRESENT_POSITION = 37
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 1023  # Refer to the CCW Angle Limit of product eManual
    BAUDRATE = 1000000  # Default Baudrate of XL-320 is 1Mbps

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID1 = 1
DXL_ID2 = 2
DXL_ID3 = 3
# Use the actual  assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME_M = (
    "COM4"  # "/dev/ttyUSB0"#http://192.168.0.189:2101/"#"COM3"#'/dev/ttyUSB0'
)
DEVICENAME_S = (
    "COM3"  # "/dev/ttyUSB1"#http://192.168.0.189:2101/"#"COM3"#'/dev/ttyUSB0'
)

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# DXL_MOVING_STATUS_THRESHOLD = 200    # Dynamixel moving status threshold

index = 0
dxl_goal_position1 = [DXL_MINIMUM_POSITION_VALUE1, DXL_MAXIMUM_POSITION_VALUE1]
dxl_goal_position2 = [
    DXL_MINIMUM_POSITION_VALUE2,
    DXL_MAXIMUM_POSITION_VALUE2,
]  # Goal position
# Goal position
dxl_present_current = 0  # current value
dxl_Current_Limit = 0
St = ""
# Ang_=read_angles(stt)
# Ang=[round(x[1]) for x in Ang_]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandlerm = PortHandler(DEVICENAME_M)
portHandler = PortHandler(DEVICENAME_S)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler1 = PacketHandler(PROTOCOL_VERSION)
packetHandler2 = PacketHandler(PROTOCOL_VERSION)
packetHandler3 = PacketHandler(PROTOCOL_VERSION)
packetHandler4 = PacketHandler(PROTOCOL_VERSION)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Open port
if portHandlerm.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandlerm.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while 1:
    if vel != "":
        break
    # St = input("""Input Desired State of control
    #             A : Direct Angle Position Control
    #             S : Sweep Action Position Control
    #             K : Key pad Motion Control
    #             Input = """)
    vel = input(
        """Input 1 for velocity control
                """
    )  # K : Key pad Motion Control

    if vel == "1":
        print("State to VM")
        # St="VM"
    else:
        St = ""

# ##Master
if vel == "1":
    # Enable velocity Control 2
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(
        portHandlerm, DXL_ID2, ADDR_OPERATING_MODE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("xxm540m1")
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
        print("xxm540m2")
    else:
        print("Velocity Control has been successfully enabled for xm5402")

# #Enable Dynamixel Torque xm5402
# dxl_comm_result2, dxl_error2 = packetHandler1.write1ByteTxRx(portHandlerm, DXL_ID2, ADDR_TORQUE_ENABLE2, TORQUE_ENABLE)
# if dxl_comm_result2 != COMM_SUCCESS:
#     print("%s" % packetHandler1.getTxRxResult(dxl_comm_result2))
# elif dxl_error2 != 0:
#     print("%s" % packetHandler1.getRxPacketError(dxl_error2))
# else:
#     print("Dynamixel xm540 has been successfully connected")

if vel == "1":
    # Enable velocity Control 1
    dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(
        portHandlerm, DXL_ID3, ADDR_OPERATING_MODE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))
    else:
        print("Velocity Control has been successfully enabled for xm54033")
# #Enable Dynamixel Torque xm5402
# dxl_comm_result2, dxl_error2 = packetHandler2.write1ByteTxRx(portHandlerm, DXL_ID3, ADDR_TORQUE_ENABLE2, TORQUE_ENABLE)
# if dxl_comm_result2 != COMM_SUCCESS:
#     print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
# elif dxl_error2 != 0:
#     print("%s" % packetHandler2.getRxPacketError(dxl_error2))
# else:
#     print("Dynamixel xm540 has been successfully connected")

# # Enable Dynamixel Torque PRO
# dxl_comm_result1, dxl_error1 = packetHandler1.write1ByteTxRx(portHandlerm, DXL_ID1, ADDR_TORQUE_ENABLE1, TORQUE_ENABLE)
# if dxl_comm_result1 != COMM_SUCCESS:
#     print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
# elif dxl_error1 != 0:
#     print("%s" % packetHandler1.getRxPacketError(dxl_error1))
# else:
#     print("Dynamixel PRO has been successfully connected")
# print("xxm540m")
# if vel == "1":
#     # Enable velocity Control 2
#     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandlerm, DXL_ID2, ADDR_OPERATING_MODE, TORQUE_ENABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("xxm540m")
#         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler2.getRxPacketError(dxl_error))
#         print("xxm540m2")
#     else:
#         print("Velocity Control has been successfully enabled for xm5402")


## Slave
if vel == "1":
    # Enable velocity Control 1
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(
        portHandler, DXL_ID1, ADDR_OPERATING_MODE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))
    else:
        print("Velocity Control has been successfully enabled for PRO")
# Enable Dynamixel Torque PRO
dxl_comm_result1, dxl_error1 = packetHandler1.write1ByteTxRx(
    portHandler, DXL_ID1, ADDR_TORQUE_ENABLE1, TORQUE_ENABLE
)
if dxl_comm_result1 != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
elif dxl_error1 != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_error1))
else:
    print("Dynamixel PRO has been successfully connected")

# if vel == "1":
#     # Enable velocity Control 3
#     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL_ID2, ADDR_OPERATING_MODE, TORQUE_ENABLE)
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler2.getRxPacketError(dxl_error))
#     else:
#         print("Velocity Control has been successfully enabled for xm540")
#
# Enable Dynamixel Torque Pro2
dxl_comm_result2, dxl_error2 = packetHandler2.write1ByteTxRx(
    portHandler, DXL_ID2, ADDR_TORQUE_ENABLE1, TORQUE_ENABLE
)
if dxl_comm_result2 != COMM_SUCCESS:
    print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
elif dxl_error2 != 0:
    print("%s" % packetHandler2.getRxPacketError(dxl_error2))
else:
    print("Dynamixel Pro 2 has been successfully connected")

if vel == "1":
    # Enable velocity Control xm5403
    dxl_comm_result, dxl_error = packetHandler3.write1ByteTxRx(
        portHandler, DXL_ID3, ADDR_OPERATING_MODE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler3.getRxPacketError(dxl_error))
    else:
        print("Velocity Control has been successfully enabled for 3")

# Enable Dynamixel Torque xm540
dxl_comm_result3, dxl_error3 = packetHandler3.write1ByteTxRx(
    portHandler, DXL_ID3, ADDR_TORQUE_ENABLE1, TORQUE_ENABLE
)
if dxl_comm_result3 != COMM_SUCCESS:
    print("%s" % packetHandler3.getTxRxResult(dxl_comm_result3))
elif dxl_error3 != 0:
    print("%s" % packetHandler3.getRxPacketError(dxl_error3))
else:
    print("Dynamixel Base Pro has been successfully connected")


def get_key():
    first_char = getch()
    if first_char == "\x1b":
        return {"[A": "up", "[B": "down", "[C": "right", "[D": "left"}[
            getch() + getch()
        ]
    else:
        return first_char


def setInital():
    key1 = 0
    key2 = 180
    key3 = 180
    print("Pro Intial Angle=", key1)
    print("xm540 Intial Angle=", key2)
    # wipe
    # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,d1[int(float(key1))])# d[int(float(key))])# d[d[int(float(key))]])
    # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2,d2[int(float(key2))])# d[int(float(key))])# d[d[int(float(key))]])
    wipe(key1, key2, key3)
    return key1, key2, key3


def rounderror(val, m):
    div = m / 245.0
    print("div", div)
    v = round(float(val / div))
    # print(v)
    v = v * m

    return v


def SetZero():
    key1, key2, key3 = 0, 180, 0

    dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
        portHandler, DXL_ID1, ADDR_GOAL_POSITION1, d1[int(float(key1))]
    )  # d[int(float(key))])# d[d[int(float(key))]])
    dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
        portHandler, DXL_ID2, ADDR_GOAL_POSITION2, d2[int(float(key2))]
    )  # d[int(float(key))])# d[d[int(float(key))]])
    dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
        portHandler, DXL_ID3, ADDR_GOAL_POSITION2, d2[int(float(key3))]
    )  # d[int(float(key))])# d[d[int(float(key))]])
    return key1, key2, key3


from collections import deque

queue = deque([])


def get_seq(PR, name):
    # data_lines = [ for seq_id in Ang_]
    # open output file for writing
    with open("./angles/lowpass_" + str(len(PR)) + name + ".txt", "w") as filehandle:
        json.dump(PR, filehandle)


def que(sp):
    # print(len(queue))
    if len(queue) < 2:
        queue.append(sp)
        return 0
    else:
        ch = get_change(queue[1], queue.popleft())
        return ch


# for a in [2,5,8,9,10]:
#     print(que(a))


def get_change(current, previous):
    if current == previous:
        return 0
    try:
        return (abs(current - previous) / previous) * 100.0
    except ZeroDivisionError:
        return float("inf")


def LiveLPF(y, live_sosfilter):
    ##Live low pass filter

    # simulate live filter - passing values one by one
    y_live_sosfilt = live_sosfilter(y)  # my fonction to append my arrays data[0:i]
    # y_live_sosfilt_.append(y_live_sosfilt)
    return y_live_sosfilt


def liveplot(data, y_live_sosfilt, name, time_line):
    import matplotlib.pyplot as plt
    import numpy as np

    plt.figure(figsize=[16, 10])
    plt.plot(time_line, data, label="Noisy signal")
    # plt.plot(ts, y_scipy_sosfilt, lw=2, label="SciPy lfilter")
    plt.plot(time_line, y_live_sosfilt, lw=5, label="LiveLFilter" + name)

    plt.legend(loc="lower center", bbox_to_anchor=[0.5, 1], ncol=2, fontsize="smaller")
    plt.xlabel("time.time()")
    plt.ylabel("Amplitude")
    plt.tight_layout()
    # plt.pause(0.25)
    plt.savefig("./src/fig/" + name + ".png")
    plt.show()


while 1:
    print("Enter to start")
    if getch() == chr(0x1B):
        break
    # St="S"
    while 1:
        if (
            St == "A"
            or St == "S"
            or St == "K"
            or St == "W"
            or St == "I"
            or St == "SM"
            or St == "VM"
        ):
            break
        # St = input("""Input Desired State of control
        #             A : Direct Angle Position Control
        #             S : Sweep Action Position Control
        #             K : Key pad Motion Control
        #             Input = """)
        St = input(
            """Input Desired State of control
                   SM : Position 3D Mouse Control
                   VM : Velocity 3D Mouse Control
                   LF : Leader follower Control
                    E : END
                  LFP : Leader Follower Position Control"""
        )  # K : Key pad Motion Control

        if St == "E":
            break

        if St != "":
            break

    # Write goal position
    if (
        MY_DXL == "XL320"
    ):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index]
        )

    elif St == "E":
        break
    else:

        if State_[St] == State_["S"]:
            print("State:", State_[St])
            # A1=45
            # A2=0
            # FREQ=0.1
            # Dur=10
            # stt=0.015
            # v=2
            # A1=0
            # A2=30
            # FREQ=0.5
            # Dur=10
            # stt=0.015
            # AF = 4294967295
            # W =0
            # v=8
            # A1=30
            # A2=0
            # FREQ=0.1
            # Dur=15
            # stt=0.015
            # while 1:
            #     A1= input("Amplitude ")
            #     # A1=45
            #     if A1 != "":
            #         break
            AF = 4294967295
            # print('''
            # Amplitude      %11d
            # Neutral Angle  %11d
            # FREQ in Hz     %11.4f
            # Duration in ms %11d
            # Time step in ms%11.4f '''%(A1,A2,FREQ,Dur,stt))

            # while 1:
            #     A2 = input("Neutral Angle (180)")
            #     A2=180
            #     if A2 != "":
            #         break
            # # if
            # while 1:
            #     FREQ = input("Frequency in Hz (0.5)")
            #     FREQ=0.5
            #     if FREQ != "":
            #         break
            # while 1:
            #     Dur = input("Time Duration in Seconds (10)")
            #     Dur=10
            #     if Dur != "":
            #         break
            # while 1:
            #     stt = input("Step amount (0.01)")
            #     stt=0.01
            #     if stt != "":
            #         break
            # while 1:
            #     print("Enter to start Sweep with Neutral Angle:%11d and Amplitude:%11d",A2,A1)
            #     if getch() == chr(0x1b):
            #         break

            # print("inside")
            # FREQ = 0.5

            t0 = time.time()

            while 1:
                key1 = input("Input Goal Angle  for PROs ")
                if key1 != "":
                    break
            while 1:
                key2 = input("Input Goal Angle  for xm540s ")
                if key2 != "":
                    break
            # while 1:
            #         key3 = input("Input Goal Angle  for xm540s ")
            #         if key3 != "":
            #             break
            print("Before")
            t0 = time.time()

            (
                dxl_present_position1,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read4ByteTxRx(
                portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
            )
            (
                dxl_present_position2,
                dxl_comm_result2,
                dxl_error2,
            ) = packetHandler2.read4ByteTxRx(
                portHandler, DXL_ID2, ADDR_PRESENT_POSITION2
            )
            Dur = servo_delay(
                dxl_present_position1,
                d1[int(key1)],
                dxl_present_position2,
                d2[int(key2)],
            )
            print("estimated", Dur)
            time.sleep(2)

            # signal.signal(signal.SIGALRM, sendang)
            print("375")

            # signal.setitimer(signal.ITIMER_REAL, stt, stt)

            time.sleep(5 + int(Dur))

            print("382")

            t1 = time.time()
            print("t1-t0", t1 - t0)

        elif State_[St] == State_["K"]:
            print("State:", State_[St])
            get_seq(pos_ll)
            # while 1:
            #     key1 = input("Imput PRO velocity")
            #     # A2=180
            #     if key1 != "":
            #         break
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(key1))# d[int(float(key))])# d[d[int(float(key))]])
            # while 1:
            #     print("Press arrow")
            #     if getch() == chr(0x1b):
            #         break
            # key = ''
            # keystate={}
            #
            # with Listener(on_press=on_press,on_release=on_release) as listener:
            #     listener.join()
            # while True:
            #     # key2=180
            #     # print("Press Arrow")
            #     if key == "q":
            #         break
            #     key = get_key()
            #
            #     if key == "up":
            #         key1=moveup()
            #         # keystate["up"=True
            #         print(key)
            #     elif key == "down":
            #         key1=movedown()
            #         # keystate.up=True
            #         print(key)
            #     elif key == "left":
            #         key2=moveleft()
            #         print(key)
            #     elif key == "right":
            #         key2=moveright()
            pass  #         print(key)
        elif State_[St] == State_["SM"]:
            print("State:", State_[St])
            # while 1:
            #     print("Press arrow")
            #     if getch() == chr(0x1b):
            #         break
            # key = ''
            # keystate={}

            # See http://stackoverflow.com/questions/29345325/raspberry-pyusb-gets-resource-busy#29347455
            # Run python2 as root (sudo /usr/bin/python2.7 /home/pi/pythondev/HelloSpaceNavigator.py)
            import sys
            import time
            from time import gmtime, strftime

            import usb.core
            import usb.util

            # Look for SpaceNavigator
            # dev = usb.core.find(idVendor=0x46d, idProduct=0xc626)
            # dev = usb.core.find(idVendor=0x256f, idProduct=0xc652)

            dev = usb.core.find(idVendor=0x256F, idProduct=0xC62E)
            if dev is None:
                raise ValueError("SpaceNavigator not found")
            else:
                print("SpaceNavigator found")
                print("dev")

            dev.reset()
            # Don't need all this but may want it for a full implementation

            cfg = dev.get_active_configuration()
            print("cfg is ", cfg)
            intf = cfg[(0, 0)]
            print("intf is ", intf)
            ep = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
                == usb.util.ENDPOINT_IN,
            )
            print("ep is ", ep)

            reattach = False
            if dev.is_kernel_driver_active(0):
                reattach = True
                dev.detach_kernel_driver(0)

            ep_in = dev[0][(0, 0)][0]
            ep_out = dev[0][(0, 0)][1]

            print("")
            print("Exit by pressing any button on the SpaceNavigator")
            print("")

            def to_int16(y1, y2):
                x = (y1) | (y2 << 8)
                if x >= 32768:
                    x = -(65536 - x)
                return x

            run = True
            while run:
                try:
                    data = dev.read(
                        ep_in.bEndpointAddress, ep_in.wMaxPacketSize
                    )  # bLength, 0)
                    # raw data
                    # print (data)

                    # print it correctly T: x,y,z R: x,y,z
                    if data[0] == 1:
                        # translation packet
                        tx = to_int16(data[1], data[2])
                        ty = to_int16(data[3], data[4])
                        tz = to_int16(data[5], data[6])

                        # if data[2] > 127:
                        #     tx -= 65536
                        # if data[4] > 127:
                        #     ty -= 65536
                        # if data[6] > 127:
                        #     tz -= 65536
                        # print (to_int16(data[7],data[8]))

                        # if data[0] == 1:
                        # rotation packet
                        rx = to_int16(data[7], data[8])
                        ry = to_int16(data[9], data[10])
                        rz = to_int16(data[11], data[12])

                        # rx = data[7] + (data[8]*256)
                        # ry = data[9] + (data[10]*256)
                        # rz = data[11] + (data[12]*256)

                        # if data[7] > 127:
                        #     rx -= 65536
                        # if data[9] > 127:
                        #     ry -= 65536
                        # if data[11] > 127:
                        #     rz -= 65536
                        spm = [tx, ty, tz, rx, ry, rz]

                        # print(spm)
                        # print( "X:",tx, "Y:", ty, "Z:", tz,"roll:",rx, "pitch:", ry, "yaw:", rz)
                        # if tz< - 180 and ty> 180:
                        #     key1=moveup()
                        #     key2=moveleft()
                        #     print("up,back")
                        # elif tz< -180 and ty< -180:
                        #     key1=moveup()
                        #     key2=moveright()
                        #     print("up,forward")
                        # elif tz>180 and ty< - 180:
                        #     key1=movedown()
                        #     key2=moveright()
                        #     print("down,forward")
                        # elif tz>180 and ty> 180:
                        #     key1=movedown()
                        #     key2=moveleft()
                        #     print("down,back")
                        # if tz< -180:#found["up"]:
                        #     key1=moveup()
                        #     print("up")
                        # elif tz>180 :#found["down"]:
                        #     key1=movedown()
                        #     print("down")
                        # elif ty< - 180:#found["right"]:
                        #     key2=moveright()
                        #     # print(key)
                        #     print("forward")
                        # elif ty> 180:#found["left"]:
                        #
                        #     key2=moveleft()
                        #     print("back")
                        # elif rz>180:#found["right"]:
                        #     # key2=moveright()
                        #     # print(key)
                        #     print("right")
                        # elif rz< -180:#found["left"]:
                        #
                        #     # key2=moveleft()
                        #     print("left")
                        if spm[5] > 180:  # found["right"]:
                            key3 = moveright2()
                            # print(key)
                            print("right")
                            if spm[1] < -180 and spm[3] > 180:
                                key1 = moveup()
                                key2 = moveleft()
                                print("down,forward")
                            elif spm[1] < -180 and spm[3] < -180:
                                key1 = moveup()
                                key2 = moveright()
                                print("down,back")
                            elif spm[1] > 180 and spm[3] < -180:
                                key1 = movedown()
                                key2 = moveright()
                                print("up,back")
                            elif spm[1] > 180 and spm[3] > 180:
                                key1 = movedown()
                                key2 = moveleft()
                                print("up,forward")
                            elif spm[1] < -180:  # found["up"]:
                                key1 = moveup()
                                print("down")
                            elif spm[1] > 180:  # found["down"]:
                                key1 = movedown()
                                print("up")
                            elif spm[3] < -180:  # found["right"]:
                                key2 = moveright()
                                # print(key)
                                print("back")
                            elif spm[3] > 180:  # found["left"]:

                                key2 = moveleft()
                                print("forward")
                        elif spm[5] < -180:  # found["left"]:

                            key3 = moveleft2()
                            print("left")
                            if spm[1] < -180 and spm[3] > 180:
                                key1 = moveup()
                                key2 = moveleft()
                                print("down,forward")
                            elif spm[1] < -180 and spm[3] < -180:
                                key1 = moveup()
                                key2 = moveright()
                                print("down,back")
                            elif spm[1] > 180 and spm[3] < -180:
                                key1 = movedown()
                                key2 = moveright()
                                print("up,back")
                            elif spm[1] > 180 and spm[3] > 180:
                                key1 = movedown()
                                key2 = moveleft()
                                print("up,forward")
                            elif spm[1] < -180:  # found["up"]:
                                key1 = moveup()
                                print("down")
                            elif spm[1] > 180:  # found["down"]:
                                key1 = movedown()
                                print("up")
                            elif spm[3] < -180:  # found["right"]:
                                key2 = moveright()
                                # print(key)
                                print("back")
                            elif spm[3] > 180:  # found["left"]:

                                key2 = moveleft()
                                print("forward")

                        elif spm[1] < -180 and spm[3] > 180:
                            key1 = moveup()
                            key2 = moveleft()
                            print("down,forward")
                        elif spm[1] < -180 and spm[3] < -180:
                            key1 = moveup()
                            key2 = moveright()
                            print("down,back")
                        elif spm[1] > 180 and spm[3] < -180:
                            key1 = movedown()
                            key2 = moveright()
                            print("up,back")
                        elif spm[1] > 180 and spm[3] > 180:
                            key1 = movedown()
                            key2 = moveleft()
                            print("up,forward")
                        elif spm[1] < -180:  # found["up"]:
                            key1 = moveup()
                            print("down")
                        elif spm[1] > 180:  # found["down"]:
                            key1 = movedown()
                            print("up")
                        elif spm[3] < -180:  # found["right"]:
                            key2 = moveright()
                            # print(key)
                            print("back")
                        elif spm[3] > 180:  # found["left"]:

                            key2 = moveleft()
                            print("forward")

                        # spc(spe)
                        # if spm[1]< - 180 and spm[3]> 180:
                        #     key1=moveup()
                        #     key2=moveleft()
                        #     print("up,back")
                        # elif spm[1]< -180 and spm[3]< -180:
                        #     key1=moveup()
                        #     key2=moveright()
                        #     print("up,forward")
                        # elif spm[1]>180 and spm[3]< - 180:
                        #     key1=movedown()
                        #     key2=moveright()
                        #     print("down,forward")
                        # elif spm[1]>180 and spm[3]> 180:
                        #     key1=movedown()
                        #     key2=moveleft()
                        #     print("down,back")
                        # elif spm[1]< -180:#found["up"]:
                        #     key1=moveup()
                        #     print("up")
                        # elif spm[1]>180 :#found["down"]:
                        #     key1=movedown()
                        #     print("down")
                        # elif spm[3]< - 180:#found["right"]:
                        #     key2=moveright()
                        #     # print(key)
                        #     print("forward")
                        # elif spm[3]> 180:#found["left"]:
                        #
                        #     key2=moveleft()
                        #     print("back")
                        # elif spm[5]>180:#found["right"]:
                        #     # key2=moveright()
                        #     # print(key)
                        #     print("right")
                        # elif spm[5]< -180:#found["left"]:
                        #
                        #     # key2=moveleft()
                        #     print("left")
                        # # spc(spe)
                    if spm[2] > 250:
                        print("reset!!!")
                        SetZero()
                        # key1,key2,key3=setInital()
                    if data[0] == 3 and data[1] == 0:
                        # button packet - exit on the release
                        run = False

                except usb.core.USBError as e:
                    data = None
                    if e.args[1] == ("Operation timed out"):
                        continue
                    else:
                        print("USB error", e)
                except:
                    print("read failed")
            # end while
            usb.util.dispose_resources(dev)

            if reattach:
                dev.attach_kernel_driver(0)

            # while True:
            #     # key2=180
            #     # print("Press Arrow")
            #     if key == "q":
            #         break
            #     key = get_key()
            #
            #     if key == "up":
            #         key1=moveup()
            #         # keystate["up"=True
            #         print(key)
            #     elif key == "down":
            #         key1=movedown()
            #         # keystate.up=True
            #         print(key)
            #     elif key == "left":
            #         key2=moveleft()
            #         print(key)
            #     elif key == "right":
            #         key2=moveright()
            #         print(key)

            pass  # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2,d2[int(float(key2))])# d[int(float(key))])# d[d[int(float(key))]])
            # elif State_[St] == State_["SM"]:
            (
                dxl_present_position1,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID1, ADDR_PRESENT_POSITION2
            )
            (
                dxl_present_position2,
                dxl_comm_result2,
                dxl_error2,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID2, ADDR_PRESENT_POSITION2
            )
            (
                dxl_present_position3,
                dxl_comm_result3,
                dxl_error3,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID3, ADDR_PRESENT_POSITION2
            )
            XPL1 = []
            XPL2 = []
            XPL3 = []
            xm540L1 = []
            xm540L2 = []
            xm540L3 = []

            XPP1 = []
            XPP2 = []
            XPP3 = []

            XL1 = []
            XL2 = []
            XL3 = []
            time_line = []
            times = []
            # dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_POSITION1)
            # dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, DXL_ID1, 20)
            while 1:
                print("Enter")
                if getch() == chr(0x1B):
                    break

                import scipy.signal

                d = 2
                fs = 30
                Wn = 2
                # d=3
                # fs = 60
                # Wn=2
                # import pandas as pd
                # from sklearn.metrics import mean_absolute_error as mae
                from utils.digitalfilter import LiveSosFilter

                # sos1 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter1 = LiveSosFilter(sos1)
                # sos2 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter2 = LiveSosFilter(sos2)
                # sos3 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter3 = LiveSosFilter(sos3)

                sosp1 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl1 = LiveSosFilter(sosp1)
                sosp2 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl2 = LiveSosFilter(sosp2)
                sosp3 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl3 = LiveSosFilter(sosp3)
                run = True
                try:
                    iii = 0
                    while run:
                        start_time = time.time()
                        (
                            dxl_present_position1,
                            dxl_comm_result1,
                            dxl_error1,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID1, ADDR_PRESENT_POSITION2
                        )

                        (
                            dxl_present_position2,
                            dxl_comm_result2,
                            dxl_error2,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID2, ADDR_PRESENT_POSITION2
                        )
                        (
                            dxl_present_position3,
                            dxl_comm_result3,
                            dxl_error3,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID3, ADDR_PRESENT_POSITION2
                        )
                        b = 2048  # bias
                        start_time1 = time.time()
                        print("time1", start_time1 - start_time)
                        xp = toSigned32(dxl_present_position1) - b
                        xp2 = toSigned32(dxl_present_position2) - b
                        xp3 = toSigned32(dxl_present_position3) - b
                        # print("xm5401 ",xp,"xm5402 ",xp2,"xm5402 ",xp3)
                        # xm5401=a2(d2,dxl_present_position1)
                        # # xm5402=a2(d2,dxl_present_position2)
                        # XPL1.append(xp)
                        # XPL2.append(xp2)
                        # XPL3.append(xp3)

                        # xm5401=LiveLPF(xp,live_sosfilter)
                        # xm5402=LiveLPF(xp2,live_sosfilter)
                        # xm5403=LiveLPF(xp3,live_sosfilter)
                        # xm5401=live_sosfilter1(xp)
                        # xm5402=live_sosfilter2(xp2)
                        # xm5403=live_sosfilter3(xp3)

                        # xm540L1.append(xm5401)
                        # xm540L2.append(xm5402)
                        # xm540L3.append(xm5403)

                        # print("xm5401f ",xm5401,"xm5402f ",xm5402,"xm5402f ",xm5403)

                        # liveplot(XPL1,xm540L1,"1")

                        # print("moveprov----","p1",p1,"p",p,"sp",sp)
                        # print("xm540value  ",dxl_present_position1,"  signed  ",xp,"  Angle  " , xm5401)
                        # print("xm540value  ",dxl_present_position2,"  signed  ",xp2,"  Angle  " , xm5402)

                        # p=pos2ang(p1)
                        start_time2 = time.time()
                        print("time2", start_time2 - start_time1)

                        # print("pos1",sp)
                        # print("pos2",sp2)
                        print("--------------")

                        (
                            dxl_present_positionp1,
                            dxl_comm_result1,
                            dxl_error1,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
                        )
                        (
                            dxl_present_positionp2,
                            dxl_comm_result2,
                            dxl_error2,
                        ) = packetHandler2.read4ByteTxRx(
                            portHandler, DXL_ID2, ADDR_PRESENT_POSITION1
                        )
                        (
                            dxl_present_positionp3,
                            dxl_comm_result3,
                            dxl_error3,
                        ) = packetHandler2.read4ByteTxRx(
                            portHandler, DXL_ID3, ADDR_PRESENT_POSITION1
                        )
                        start_time3 = time.time()
                        print("time3", start_time3 - start_time2)

                        sp = toSigned32(dxl_present_positionp1)
                        sp2 = toSigned32(dxl_present_positionp2)
                        sp3 = toSigned32(dxl_present_positionp3)

                        p1 = pos2ang(dxl_present_positionp1)
                        p2 = pos2ang(dxl_present_positionp2)
                        p3 = pos2ang(dxl_present_positionp3)

                        # print("moveprov----","p1",p1,"p",p,"sp",sp)
                        # print("value  ",dxl_present_positionp1,"  signed  ",sp,"  Angle  " , p1)
                        # print("value  ",dxl_present_positionp2,"  signed  ",sp2,"  Angle  " , p2)

                        # p=pos2ang(p1)
                        start_time4 = time.time()
                        print("time4", start_time4 - start_time3)

                        # print("pos1",sp)
                        # print("pos2",sp2)
                        print("--------------")
                        multiplier = 245
                        key1 = rounderror(xp, multiplier)

                        key2 = rounderror(xp2, multiplier)
                        key3 = rounderror(xp3, multiplier)

                        # key1 = rounderror(xm5401,multiplier)
                        # #
                        # key2 = rounderror(xm5402,multiplier)
                        # key3 = rounderror(xm5403,multiplier)
                        start_time5 = time.time()
                        print("time5", start_time5 - start_time4)
                        XPP1.append(key1)
                        XPP2.append(key2)
                        XPP3.append(key3)
                        start_time6 = time.time()
                        print("time6", start_time6 - start_time5)
                        x1 = live_sosfilterl1(key1)
                        x2 = live_sosfilterl2(key2)
                        x3 = live_sosfilterl3(key3)
                        start_time7 = time.time()
                        print("time7", start_time7 - start_time6)
                        # x1=key1
                        # x2=key2
                        # x3=key3
                        print("xm5401 ", x1, "xm5402 ", x2, "xm5402 ", x3)

                        # print("xm5401 ",xm5401,"xm5402 ",xm5402,"xm5402 ",xm5403)
                        XL1.append(x1)
                        XL2.append(x2)
                        XL3.append(x3)
                        print("------x3", x3)
                        # key2=movexm540v(key2)
                        start_time8 = time.time()
                        print("time8", start_time8 - start_time7)
                        # key1=moveprop(x1,1)
                        # key2=moveprop(x2,2)
                        # key3=moveprop(x3,3)
                        key1 = moveprop(key1, 1)
                        key2 = moveprop(key2, 2)
                        key3 = moveprop(key3, 3)
                        start_time9 = time.time()
                        print("time9", start_time9 - start_time8)

                        print(iii)
                        # print("Pro1",pos2ang(key1),"Pro2",pos2ang(key3))
                        #################################################################

                        # if getch() == chr(0x20):
                        #     # print("beforekey1",key1,"key2",key3)
                        #     print("key1",toSigned32(key1),"key2",toSigned32(key3))
                        # break
                        # run = False
                        finish_time = time.time()
                        time_line.append(finish_time)
                        # break
                        times.append(
                            [
                                start_time,
                                start_time1,
                                start_time2,
                                start_time3,
                                start_time4,
                                start_time5,
                                start_time6,
                                start_time7,
                                start_time8,
                                start_time9,
                                finish_time,
                            ]
                        )
                        iii = iii + 1
                        if iii >= 2000:
                            print(
                                iii,
                                "---------------------------------------------------------",
                            )
                            # liveplot(XPL1,xm540L1,"1"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            # liveplot(XPL2,xm540L2,"2"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            # liveplot(XPL3,xm540L3,"3"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            get_seq(times, str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            liveplot(
                                XPP1,
                                XL1,
                                "p1"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            liveplot(
                                XPP2,
                                XL2,
                                "p2"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            liveplot(
                                XPP3,
                                XL3,
                                "p3"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            break
                except KeyboardInterrupt:
                    # liveplot(XPL1,xm540L1,"1"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    # liveplot(XPL2,xm540L2,"2"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    # liveplot(XPL3,xm540L3,"3"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    liveplot(
                        XPP1,
                        XL1,
                        "p1" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )
                    liveplot(
                        XPP2,
                        XL2,
                        "p2" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )
                    liveplot(
                        XPP3,
                        XL3,
                        "p3" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )

                    break

        elif State_[St] == State_["LFP"]:
            (
                dxl_present_position1,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID1, ADDR_PRESENT_POSITION2
            )
            (
                dxl_present_position2,
                dxl_comm_result2,
                dxl_error2,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID2, ADDR_PRESENT_POSITION2
            )
            (
                dxl_present_position3,
                dxl_comm_result3,
                dxl_error3,
            ) = packetHandler1.read4ByteTxRx(
                portHandlerm, DXL_ID3, ADDR_PRESENT_POSITION2
            )
            XPL1 = []
            XPL2 = []
            XPL3 = []
            xm540L1 = []
            xm540L2 = []
            xm540L3 = []

            XPP1 = []
            XPP2 = []
            XPP3 = []

            XL1 = []
            XL2 = []
            XL3 = []
            time_line = []
            times = []
            # dxl_present_position1, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_POSITION1)
            # dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(portHandler, DXL_ID1, 20)
            while 1:
                print("Enter")
                if getch() == chr(0x1B):
                    break

                import scipy.signal

                d = 2
                fs = 30
                Wn = 2
                b = 2048
                # d=3
                # fs = 60
                # Wn=2
                # import pandas as pd
                # from sklearn.metrics import mean_absolute_error as mae
                from utils.digitalfilter import LiveSosFilter

                # sos1 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter1 = LiveSosFilter(sos1)
                # sos2 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter2 = LiveSosFilter(sos2)
                # sos3 = scipy.signal.iirfilter(d, Wn=Wn, fs=fs, btype="low",
                #                     ftype="butter", output="sos")
                # live_sosfilter3 = LiveSosFilter(sos3)

                sosp1 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl1 = LiveSosFilter(sosp1)
                sosp2 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl2 = LiveSosFilter(sosp2)
                sosp3 = scipy.signal.iirfilter(
                    d, Wn=Wn, fs=fs, btype="low", ftype="butter", output="sos"
                )

                live_sosfilterl3 = LiveSosFilter(sosp3)
                run = True
                try:
                    iii = 0
                    while run:
                        start_time = time.time()
                        (
                            dxl_present_position1,
                            dxl_comm_result1,
                            dxl_error1,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID1, ADDR_PRESENT_POSITION2
                        )
                        start_time1 = time.time()
                        print("time1", start_time1 - start_time)
                        xp = toSigned32(dxl_present_position1) - b
                        start_time2 = time.time()
                        print("time2", start_time2 - start_time1)
                        (
                            dxl_present_positionp1,
                            dxl_comm_result1,
                            dxl_error1,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
                        )
                        start_time3 = time.time()
                        print("time3", start_time3 - start_time2)

                        sp = toSigned32(dxl_present_positionp1)

                        p1 = pos2ang(dxl_present_positionp1)
                        start_time4 = time.time()
                        print("time4", start_time4 - start_time3)

                        # print("pos1",sp)
                        # print("pos2",sp2)
                        print("--------------")
                        multiplier = 245
                        key1 = rounderror(xp, multiplier)
                        start_time5 = time.time()
                        print("time5", start_time5 - start_time4)
                        XPP1.append(key1)
                        start_time6 = time.time()
                        print("time6", start_time6 - start_time5)
                        x1 = live_sosfilterl1(key1)
                        start_time7 = time.time()
                        print("time7", start_time7 - start_time6)
                        # x1=key1
                        # x2=key2
                        # x3=key3
                        # print("xm5401 ",x1,"xm5402 ",x2,"xm5402 ",x3)

                        # print("xm5401 ",xm5401,"xm5402 ",xm5402,"xm5402 ",xm5403)
                        XL1.append(x1)
                        start_time8 = time.time()
                        print("time8", start_time8 - start_time7)
                        # key1=moveprop(x1,1)
                        # key2=moveprop(x2,2)
                        # key3=moveprop(x3,3)
                        key1 = moveprop(key1, 1)

                        start_time9 = time.time()
                        print("time9", start_time9 - start_time8)

                        print(iii)

                        (
                            dxl_present_position2,
                            dxl_comm_result2,
                            dxl_error2,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID2, ADDR_PRESENT_POSITION2
                        )
                        (
                            dxl_present_position3,
                            dxl_comm_result3,
                            dxl_error3,
                        ) = packetHandler1.read4ByteTxRx(
                            portHandlerm, DXL_ID3, ADDR_PRESENT_POSITION2
                        )
                        b = 2048  # bias

                        xp2 = toSigned32(dxl_present_position2) - b
                        xp3 = toSigned32(dxl_present_position3) - b
                        # print("xm5401 ",xp,"xm5402 ",xp2,"xm5402 ",xp3)
                        # xm5401=a2(d2,dxl_present_position1)
                        # # xm5402=a2(d2,dxl_present_position2)
                        # XPL1.append(xp)
                        # XPL2.append(xp2)
                        # XPL3.append(xp3)

                        # xm5401=LiveLPF(xp,live_sosfilter)
                        # xm5402=LiveLPF(xp2,live_sosfilter)
                        # xm5403=LiveLPF(xp3,live_sosfilter)
                        # xm5401=live_sosfilter1(xp)
                        # xm5402=live_sosfilter2(xp2)
                        # xm5403=live_sosfilter3(xp3)

                        # xm540L1.append(xm5401)
                        # xm540L2.append(xm5402)
                        # xm540L3.append(xm5403)

                        # print("xm5401f ",xm5401,"xm5402f ",xm5402,"xm5402f ",xm5403)

                        # liveplot(XPL1,xm540L1,"1")

                        # print("moveprov----","p1",p1,"p",p,"sp",sp)
                        # print("xm540value  ",dxl_present_position1,"  signed  ",xp,"  Angle  " , xm5401)
                        # print("xm540value  ",dxl_present_position2,"  signed  ",xp2,"  Angle  " , xm5402)

                        # p=pos2ang(p1)

                        # print("pos1",sp)
                        # print("pos2",sp2)
                        print("--------------")

                        (
                            dxl_present_positionp2,
                            dxl_comm_result2,
                            dxl_error2,
                        ) = packetHandler2.read4ByteTxRx(
                            portHandler, DXL_ID2, ADDR_PRESENT_POSITION1
                        )
                        (
                            dxl_present_positionp3,
                            dxl_comm_result3,
                            dxl_error3,
                        ) = packetHandler2.read4ByteTxRx(
                            portHandler, DXL_ID3, ADDR_PRESENT_POSITION1
                        )

                        sp2 = toSigned32(dxl_present_positionp2)
                        sp3 = toSigned32(dxl_present_positionp3)

                        p2 = pos2ang(dxl_present_positionp2)
                        p3 = pos2ang(dxl_present_positionp3)

                        # print("moveprov----","p1",p1,"p",p,"sp",sp)
                        # print("value  ",dxl_present_positionp1,"  signed  ",sp,"  Angle  " , p1)
                        # print("value  ",dxl_present_positionp2,"  signed  ",sp2,"  Angle  " , p2)

                        # p=pos2ang(p1)

                        key2 = rounderror(xp2, multiplier)
                        key3 = rounderror(xp3, multiplier)

                        # key1 = rounderror(xm5401,multiplier)
                        # #
                        # key2 = rounderror(xm5402,multiplier)
                        # key3 = rounderror(xm5403,multiplier)

                        XPP2.append(key2)
                        XPP3.append(key3)

                        x2 = live_sosfilterl2(key2)
                        x3 = live_sosfilterl3(key3)

                        XL2.append(x2)
                        XL3.append(x3)
                        print("------x3", x3)
                        # key2=movexm540v(key2)

                        key2 = moveprop(key2, 2)
                        key3 = moveprop(key3, 3)

                        # print("Pro1",pos2ang(key1),"Pro2",pos2ang(key3))
                        #################################################################

                        # if getch() == chr(0x20):
                        #     # print("beforekey1",key1,"key2",key3)
                        #     print("key1",toSigned32(key1),"key2",toSigned32(key3))
                        # break
                        # run = False
                        finish_time = time.time()
                        time_line.append(finish_time)
                        # break
                        times.append(
                            [
                                start_time,
                                start_time1,
                                start_time2,
                                start_time3,
                                start_time4,
                                start_time5,
                                start_time6,
                                start_time7,
                                start_time8,
                                start_time9,
                                finish_time,
                            ]
                        )
                        iii = iii + 1
                        if iii >= 200:
                            print(
                                iii,
                                "---------------------------------------------------------",
                            )
                            # liveplot(XPL1,xm540L1,"1"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            # liveplot(XPL2,xm540L2,"2"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            # liveplot(XPL3,xm540L3,"3"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                            get_seq(times, str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn))
                            liveplot(
                                XPP1,
                                XL1,
                                "p1"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            liveplot(
                                XPP2,
                                XL2,
                                "p2"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            liveplot(
                                XPP3,
                                XL3,
                                "p3"
                                + "_d-"
                                + str(d)
                                + "_fs-"
                                + str(fs)
                                + "_Wn-"
                                + str(Wn),
                                time_line,
                            )
                            break
                except KeyboardInterrupt:
                    # liveplot(XPL1,xm540L1,"1"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    # liveplot(XPL2,xm540L2,"2"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    # liveplot(XPL3,xm540L3,"3"+"_d-"+str(d)+"_fs-"+str(fs)+"_Wn-"+str(Wn))
                    liveplot(
                        XPP1,
                        XL1,
                        "p1" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )
                    liveplot(
                        XPP2,
                        XL2,
                        "p2" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )
                    liveplot(
                        XPP3,
                        XL3,
                        "p3" + "_d-" + str(d) + "_fs-" + str(fs) + "_Wn-" + str(Wn),
                    )

                    break
        elif State_[St] == State_["MS"]:
            print("State:", State_[St])

            ##Slave
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_Profile_Velocity,1500)# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_Profile_Acceleration,30000)# d[int(float(key))])# d[d[int(float(key))]])
            #
            # # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_Profile_Velocity2,120)# d[int(float(key))])# d[d[int(float(key))]])
            # # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_Profile_Acceleration2,45)# d[int(float(key))])# d[d[int(float(key))]])
            #
            # dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(portHandler, DXL_ID3, ADDR_Profile_Velocity,1500)# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(portHandler, DXL_ID3, ADDR_Profile_Acceleration,30000)# d[int(float(key))])# d[d[int(float(key))]])

            ##Master
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandlerm, DXL_ID3, ADDR_Profile_Velocity,1500)# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandlerm, DXL_ID3, ADDR_Profile_Acceleration,30000)# d[int(float(key))])# d[d[int(float(key))]])

            # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandlerm, DXL_ID2, ADDR_Profile_Velocity2,120)# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandlerm, DXL_ID2, ADDR_Profile_Acceleration2,45)# d[int(float(key))])# d[d[int(float(key))]])

            dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
                portHandlerm, DXL_ID3, ADDR_Profile_Velocity2, 120
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
                portHandlerm, DXL_ID3, ADDR_Profile_Acceleration2, 45
            )  # d[int(float(key))])# d[d[int(float(key))]])

            (
                dxl_present_position1,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read4ByteTxRx(
                portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
            )
            dxl_homing, dxl_comm_result1, dxl_error1 = packetHandler1.read4ByteTxRx(
                portHandler, DXL_ID1, 20
            )
            while 1:
                print("Enter")
                if getch() == chr(0x1B):
                    break
                run = True
                while run:

                    (
                        dxl_present_velocity1,
                        dxl_comm_result1,
                        dxl_error1,
                    ) = packetHandler1.read4ByteTxRx(
                        portHandlerm, DXL_ID3, ADDR_PRESENT_VELOCITY2
                    )
                    (
                        dxl_present_velocity2,
                        dxl_comm_result2,
                        dxl_error2,
                    ) = packetHandler2.read4ByteTxRx(
                        portHandlerm, DXL_ID2, ADDR_PRESENT_VELOCITY2
                    )

                    sp = toSigned32(dxl_present_velocity1)
                    sp2 = toSigned32(dxl_present_velocity2)

                    # p=pos2ang(p1)

                    print("VEl1", sp)
                    print("Vel2", sp2)
                    print("--------------")
                    key3 = sp * 12
                    key1 = sp2 * 12

                    # key2=movexm540v(key2)
                    key1 = moveprov(key1)
                    key3 = moveprov3(key3)
                    # if getch() == chr(0x20):
                    #     break
                    #     run = False

        elif State_[St] == State_["VM"]:
            print("State:", State_[St])
            dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
                portHandler, DXL_ID1, ADDR_Profile_Velocity, 1500
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
                portHandler, DXL_ID1, ADDR_Profile_Acceleration, 30000
            )  # d[int(float(key))])# d[d[int(float(key))]])

            dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
                portHandler, DXL_ID2, ADDR_Profile_Velocity2, 120
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
                portHandler, DXL_ID2, ADDR_Profile_Acceleration2, 45
            )  # d[int(float(key))])# d[d[int(float(key))]])

            dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
                portHandler, DXL_ID3, ADDR_Profile_Velocity, 1500
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
                portHandler, DXL_ID3, ADDR_Profile_Acceleration, 30000
            )  # d[int(float(key))])# d[d[int(float(key))]])

            # while 1:
            #     print("Press arrow")
            #     if getch() == chr(0x1b):
            #         break
            # key = ''
            # keystate={}

            # while 1:
            #     Auto = input("Wipe y/n")
            #     # A2=180
            #     if Auto != "":
            #         break

            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(key1))# d[int(float(key))])# d[d[int(float(key))]])

            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(0))# d[int(float(key))])# d[d[int(float(key))]])

            # See http://stackoverflow.com/questions/29345325/raspberry-pyusb-gets-resource-busy#29347455
            # Run python2 as root (sudo /usr/bin/python2.7 /home/pi/pythondev/HelloSpaceNavigator.py)
            import sys
            import time
            from time import gmtime, strftime

            import usb.core
            import usb.util

            # Look for SpaceNavigator
            # dev = usb.core.find(idVendor=0x46d, idProduct=0xc626)
            # dev = usb.core.find(idVendor=0x256f, idProduct=0xc652)

            dev = usb.core.find(idVendor=0x256F, idProduct=0xC62E)
            if dev is None:
                raise ValueError("SpaceNavigator not found")
            else:
                print("SpaceNavigator found")
                print("dev")

            dev.reset()
            # Don't need all this but may want it for a full implementation

            cfg = dev.get_active_configuration()
            print("cfg is ", cfg)
            intf = cfg[(0, 0)]
            print("intf is ", intf)
            ep = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
                == usb.util.ENDPOINT_IN,
            )
            print("ep is ", ep)

            reattach = False
            if dev.is_kernel_driver_active(0):
                try:
                    reattach = True
                    dev.detach_kernel_driver(0)
                except usb.core.USBError as e:
                    sys.exit(
                        "Could not detatch kernel driver from interface({0}): {1}".format(
                            0, str(e)
                        )
                    )

            # if dev.is_kernel_driver_active(i):
            #     try:
            #         dev.detach_kernel_driver(i)

            ep_in = dev[0][(0, 0)][0]
            ep_out = dev[0][(0, 0)][1]

            print("")
            print("Exit by pressing any button on the SpaceNavigator")
            print("")

            def to_int16(y1, y2):
                x = (y1) | (y2 << 8)
                if x >= 32768:
                    x = -(65536 - x)
                return x

            run = True
            Auto = "n"
            if Auto == "y":
                # # p=range(-180,0)+[0]
                for a1 in (
                    list(range(0, 180, 20))
                    + list(range(180, 0, 20))
                    + list(range(0, -180, 20))
                    + list(range(-180, 0, 20))
                ):  # [0,20,40,80,150,300,300,300,300,300,300,150,80,20,40,0]+[0,20,40,80,150,300,300,300,300,300,300,150,80,20,40,0]*-1:

                    key1 = round(a1 * 8.28 / 2)
                    # print("key1",key1)
                    key1 = moveprov(key1)
                    print("PRO velocity", key1)
                    # else:
                    #     key1=0

                    key2 = round(a1 * 0.6686 / 2)

                    key2 = movexm540v(key2)
                    print("xm540 velocity", key2)

                    key3 = round(a1 * 8.28 / 2)
                    # print("key1",key1)
                    key3 = moveprov3(key3)

                    key1 = 0  # round(a1*8.28/2)
                    # print("key1",key1)
                    key1 = moveprov(key1)
                    print("PRO velocity", key1)
                    # else:
                    #     key1=0

                    key2 = 0  # round(a1*0.6686/2)

                    key2 = movexm540v(key2)
                    print("xm540 velocity", key2)

                    key3 = 0  # round(a1*8.28/2)
                    # print("key1",key1)
                    key3 = moveprov3(key3)

                    print("Base PRO velocity", key3)
            else:

                while 1:
                    try:
                        data = dev.read(
                            ep_in.bEndpointAddress, ep_in.wMaxPacketSize
                        )  # bLength, 0)
                        # raw data
                        # print (data)
                        spm = [0] * 6
                        # print it correctly T: x,y,z R: x,y,z

                        if data[0] == 1:
                            # translation packet
                            tx = to_int16(data[1], data[2])
                            ty = to_int16(data[3], data[4])
                            tz = to_int16(data[5], data[6])

                            # if data[2] > 127:
                            #     tx -= 65536
                            # if data[4] > 127:
                            #     ty -= 65536
                            # if data[6] > 127:
                            #     tz -= 65536
                            # print (to_int16(data[7],data[8]))

                            # if data[0] == 1:
                            # rotation packet
                            rx = to_int16(data[7], data[8])
                            ry = to_int16(data[9], data[10])
                            rz = to_int16(data[11], data[12])

                            # rx = data[7] + (data[8]*256)
                            # ry = data[9] + (data[10]*256)
                            # rz = data[11] + (data[12]*256)

                            # if data[7] > 127:
                            #     rx -= 65536
                            # if data[9] > 127:
                            #     ry -= 65536
                            # if data[11] > 127:
                            #     rz -= 65536
                            spm = [tx, ty, tz, rx, ry, rz]
                            pos_ll.append(spm)
                            # print(spm)
                            # if que(spm[1]) > 30.0 and spm[1] != 0 :

                            #
                            # if abs(spm[1]) -45 > 0:
                            key1 = round(spm[1] * 8.28 / 2)
                            # print("key1",key1)

                            print("PRO velocity", key1)
                            # else:
                            #     key1=0

                            key2 = round(spm[3] * 0.6686 / 2)

                            print("xm540 velocity", key2)

                            key3 = round(spm[5] * 8.28 / 2)
                            # print("key1",key1)
                            key2 = movexm540v(key2)
                            key1 = moveprov(key1)
                            key3 = moveprov3(key3)
                            print("Base PRO velocity", key3)
                            # if que(spm[3])>20:
                            #     print("backward",que(spm[3]))
                            # elif que(spm[3])<-20:
                            #     print("forward",que(spm[3]))
                            # elif que(spm[1])<-20:
                            #     print("up",que(spm[1]))
                            # elif que(spm[1])>20:
                            #     print("down",que(spm[1]))
                            #
                            #
                            # key3=round(spm[5]*0.6686)
                            #
                            # key3=moverightV3(key3)

                            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(key1))
                            # while 1:
                            #     key1 = input("Imput PRO velocity")
                            #     # A2=180
                            #     if key1 != "":
                            #         break
                            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(key1))# d[int(float(key))])# d[d[int(float(key))]])

                            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,int(key1))
                            # key2=round(spm[5]*)
                            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,int(key1))
                            # print(spm)
                            # print( "X:",tx, "Y:", ty, "Z:", tz,"roll:",rx, "pitch:", ry, "yaw:", rz)
                            # if tz< - 180 and ty> 180:
                            #     key1=moveup()
                            #     key2=moveleft()
                            #     print("up,back")
                            # elif tz< -180 and ty< -180:
                            #     key1=moveup()
                            #     key2=moveright()
                            #     print("up,forward")
                            # elif tz>180 and ty< - 180:
                            #     key1=movedown()
                            #     key2=moveright()
                            #     print("down,forward")
                            # elif tz>180 and ty> 180:
                            #     key1=movedown()
                            #     key2=moveleft()
                            #     print("down,back")
                            # if tz< -180:#found["up"]:
                            #     key1=moveup()
                            #     print("up")
                            # elif tz>180 :#found["down"]:
                            #     key1=movedown()
                            #     print("down")
                            # elif ty< - 180:#found["right"]:
                            #     key2=moveright()
                            #     # print(key)
                            #     print("forward")
                            # elif ty> 180:#found["left"]:
                            #
                            #     key2=moveleft()
                            #     print("back")
                            # elif rz>180:#found["right"]:
                            #     # key2=moveright()
                            #     # print(key)
                            #     print("right")
                            # elif rz< -180:#found["left"]:
                            #
                            #     # key2=moveleft()
                            #     print("left")
                            # if spm[5]>180:#found["right"]:
                            #     vel=round(spm[5]*0.917)
                            #     key3=moverightV3(vel)
                            # #     # print(key)
                            #     print("right")
                            #     if spm[1]< - 180 and spm[3]> 180:
                            #         key1=moveup()
                            #         key2=moveleft()
                            #         print("down,forward")
                            #     elif spm[1]< -180 and spm[3]< -180:
                            #         key1=moveup()
                            #         key2=moveright()
                            #         print("down,back")
                            #     elif spm[1]>180 and spm[3]< - 180:
                            #         key1=movedown()
                            #         key2=moveright()
                            #         print("up,back")
                            #     elif spm[1]>180 and spm[3]> 180:
                            #         key1=movedown()
                            #         key2=moveleft()
                            #         print("up,forward")
                            #     elif spm[1]< -180:#found["up"]:
                            #         key1=moveup()
                            #         print("down")
                            #     elif spm[1]>180 :#found["down"]:
                            #         key1=movedown()
                            #         print("up")
                            #     elif spm[3]< - 180:#found["right"]:
                            #         key2=moveright()
                            #         # print(key)
                            #         print("back")
                            #     elif spm[3]> 180:#found["left"]:
                            #
                            #         key2=moveleft()
                            # #         print("forward")
                            # elif spm[5]< -180:#found["left"]:
                            #     vel=round(spm[5]*0.917)
                            #     key3=moveleftV3(vel)
                            #     print("left")
                            #     if spm[1]< - 180 and spm[3]> 180:
                            #         key1=moveup()
                            #         key2=moveleft()
                            #         print("down,forward")
                            #     elif spm[1]< -180 and spm[3]< -180:
                            #         key1=moveup()
                            #         key2=moveright()
                            #         print("down,back")
                            #     elif spm[1]>180 and spm[3]< - 180:
                            #         key1=movedown()
                            #         key2=moveright()
                            #         print("up,back")
                            #     elif spm[1]>180 and spm[3]> 180:
                            #         key1=movedown()
                            #         key2=moveleft()
                            #         print("up,forward")
                            #     elif spm[1]< -180:#found["up"]:
                            #         key1=moveup()
                            #         print("down")
                            #     elif spm[1]>180 :#found["down"]:
                            #         key1=movedown()
                            #         print("up")
                            #     elif spm[3]< - 180:#found["right"]:
                            #         key2=moveright()
                            #         # print(key)
                            #         print("back")
                            #     elif spm[3]> 180:#found["left"]:
                            #
                            #         key2=moveleft()
                            #         print("forward")
                            #
                            # elif spm[1]< - 180 and spm[3]> 180:
                            #     key1=moveup()
                            #     key2=moveleft()
                            #     print("down,forward")
                            # elif spm[1]< -180 and spm[3]< -180:
                            #     key1=moveup()
                            #     key2=moveright()
                            #     print("down,back")
                            # elif spm[1]>180 and spm[3]< - 180:
                            #     key1=movedown()
                            #     key2=moveright()
                            #     print("up,back")
                            # elif spm[1]>180 and spm[3]> 180:
                            #     key1=movedown()
                            #     key2=moveleft()
                            #     print("up,forward")
                            # elif spm[1]< -180:#found["up"]:
                            #     key1=moveup()
                            #     print("down")
                            # elif spm[1]>180 :#found["down"]:
                            #     key1=movedown()
                            #     print("up")
                            # elif spm[3]< - 180:#found["right"]:
                            #     key2=moveright()
                            #     # print(key)
                            #     print("back")
                            # elif spm[3]> 180:#found["left"]:
                            #
                            #     key2=moveleft()
                            #     print("forward")

                            # spc(spe)
                            # if spm[1]< - 180 and spm[3]> 180:
                            #     key1=moveup()
                            #     key2=moveleft()
                            #     print("up,back")
                            # elif spm[1]< -180 and spm[3]< -180:
                            #     key1=moveup()
                            #     key2=moveright()
                            #     print("up,forward")
                            # elif spm[1]>180 and spm[3]< - 180:
                            #     key1=movedown()
                            #     key2=moveright()
                            #     print("down,forward")
                            # elif spm[1]>180 and spm[3]> 180:
                            #     key1=movedown()
                            #     key2=moveleft()
                            #     print("down,back")
                            # elif spm[1]< -180:#found["up"]:
                            #     key1=moveup()
                            #     print("up")
                            # elif spm[1]>180 :#found["down"]:
                            #     key1=movedown()
                            #     print("down")
                            # elif spm[3]< - 180:#found["right"]:
                            #     key2=moveright()
                            #     # print(key)
                            #     print("forward")
                            # elif spm[3]> 180:#found["left"]:
                            #
                            #     key2=moveleft()
                            #     print("back")
                            # elif spm[5]>180:#found["right"]:
                            #     # key2=moveright()
                            #     # print(key)
                            #     print("right")
                            # elif spm[5]< -180:#found["left"]:
                            #
                            #     # key2=moveleft()
                            #     print("left")
                            # # spc(spe)
                        if spm[2] > 250:
                            print("reset!!!")
                            # SetZero()
                            # key1,key2,key3=setInital()
                        if data[0] == 3 and data[1] == 0:
                            # St=""

                            break
                            # button packet - exit on the release
                            run = False

                    except usb.core.USBError as e:
                        data = None
                        if e.args[1] == ("Operation timed out"):
                            continue
                        else:
                            print("USB error", e)
                            # if dev.is_kernel_driver_active(i):
                            #     try:
                            #         dev.detach_kernel_driver(i)
                            #     except usb.core.USBError as e:
                            #         sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(i, str(e)))
                    # except:
                    #     print("read failed")
                    # end while
                    usb.util.dispose_resources(dev)

                    # if reattach:
                    #     dev.attach_kernel_driver(0)

            # while True:
            #     # key2=180
            #     # print("Press Arrow")
            #     if key == "q":
            #         break
            #     key = get_key()
            #
            #     if key == "up":
            #         key1=moveup()
            #         # keystate["up"=True
            #         print(key)
            #     elif key == "down":
            #         key1=movedown()
            #         # keystate.up=True
            #         print(key)
            #     elif key == "left":
            #         key2=moveleft()
            #         print(key)
            #     elif key == "right":
            #         key2=moveright()
            #         print(key)

            # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2,d2[int(float(key2))])# d[int(float(key))])# d[d[int(float(key))]])

        elif State_[St] == State_["I"]:
            print("State:", State_[St])

            # key=dxl_goal_position[index]

            # while 1:
            #         key1 = input("Input Goal Angle  for PROa ")
            #         if key1 != "":
            #             break
            # while 1:
            #         key2 = input("Input Goal Angle  for xm540a ")
            #         if key2 != "":
            #             break
            key1, key2, key3 = 0, 180, 180
            wipe(key1, key2, key3)

        elif State_[St] == State_["W"]:
            print("State:", State_[St])
            key1 = 0
            key2 = 180
            print("Pro Intial Angle=", key1)
            print("xm540 Intial Angle=", key2)
            wipe(key1, key2, key3)
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,d1[int(float(key1))])# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2,d2[int(float(key2))])# d[int(float(key))])# d[d[int(float(key))]])

            # pass
            for key1, key2, key3 in zip(
                list(range(30, 44, 2)) + [0],
                [180, 150, 210, 150, 210, 150, 210, 180],
                list(range(30, 44, 2)) + [0],
            ):
                wipe(key1, key2, key3)
            # for key1,key2 in zip([-80,-81,-82,-83,-84,-85,-86,0],[180,150,210,150,210,150,210,180]):
            # wipe(key1,key2)
            print("!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!Wiping Done!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!")
            # key1=0
            # key2=180

            # key1,key2=setInital()
        elif State_[St] == State_["SI"]:
            # break
            print("State:", State_[St])
            key1 = 0
            key2 = 180
            print("Pro Intial Angle=", key1)
            print("xm540 Intial Angle=", key2)
            wipe(key1, key2)
            while 1:
                key1 = input("Input Amplitude  for pro ")
                if key1 != "":
                    break
            Amp = 0
            wipe_sin(key1, key2, Amp)
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION1,d1[int(float(key1))])# d[int(float(key))])# d[d[int(float(key))]])
            # dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2,d2[int(float(key2))])# d[int(float(key))])# d[d[int(float(key))]])

            # pass
            # for key1,key2 in zip(list(range(80,87))+[0],[180,150,210,150,210,150,210,180]):
            #     wipe(key1,key2)
            print("!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!Sin Wiping Done!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!")

        elif State_[St] == State_["A"]:
            # key=dxl_goal_position[index]
            # while 1:
            #     key1 = input("Imput PRO velocity")
            #     # A2=180
            #     if key1 != "":
            #         break
            # dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY1,int(key1))# d[int(float(key))])# d[d[int(float(key))]])

            while 1:
                key1 = input("Input Goal Angle  for PROa ")
                if key1 != "":
                    break
            while 1:
                key2 = input("Input Goal Angle  for xm540a ")
                if key2 != "":
                    break
            while 1:
                key3 = input("Input Goal Angle  for xm540b ")
                if key3 != "":
                    break
            dxl_comm_result1, dxl_error1 = packetHandler1.write4ByteTxRx(
                portHandler, DXL_ID1, ADDR_GOAL_POSITION1, d1[int(float(key1))]
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result2, dxl_error2 = packetHandler2.write4ByteTxRx(
                portHandler, DXL_ID2, ADDR_GOAL_POSITION2, d2[int(float(key2))]
            )  # d[int(float(key))])# d[d[int(float(key))]])
            dxl_comm_result3, dxl_error3 = packetHandler3.write4ByteTxRx(
                portHandler, DXL_ID3, ADDR_GOAL_POSITION2, d1[int(float(key3))]
            )  # d[int(float(key))])# d[d[int(float(key))]])

        # print(type(key))
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, d[int(float(key))])# d[d[int(float(key))]])
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])

    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error1))
    #
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error2))

    while 1:
        print("---------read----------")
        # Read present position
        if (
            MY_DXL == "XL320"
        ):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            (
                dxl_present_position,
                dxl_comm_result,
                dxl_error,
            ) = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        # elif State_[St] == State_["VM"]:
        #     if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        #         dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler3.read2ByteTxRx(portHandler, DXL_ID3, ADDR_PRESENT_VELOCITY3)
        #     else:
        #         dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler3.read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRESENT_VELOCITY3)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % packetHandler3.getRxPacketError(dxl_error))
        #         # print("--END--")
        #         # print("S-[ID:%03d] GoalPos:%03d PresentPos:%03d" % (DXL_ID, key, dxl_present_position))
        #         #Ang_.append("S-[ID:%03d] GoalPos:%03d PresentPos:%03d" % (DXL_ID, key, dxl_present_position))
        #     if abs(key3- dxl_present_velocity) < 10:
        #         St=""
        #         # print('\n'.join(str("S==[time:%7.5f] GoalPos:%03d deg:%03d PresentPos:%03d deg:%03d "%x) for x in Ang_))
        #         print(key3,dxl_present_velocity)
        #         # Plot(Ang_,int(A2),1)
        else:
            # if State_[St] != State_["VM"]:
            (
                dxl_present_position1,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read4ByteTxRx(
                portHandler, DXL_ID1, ADDR_PRESENT_POSITION1
            )
            # dxl_present_position2, dxl_comm_result2, dxl_error2 = packetHandler2.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION2)
            (
                dxl_present_position3,
                dxl_comm_result3,
                dxl_error3,
            ) = packetHandler3.read4ByteTxRx(
                portHandler, DXL_ID3, ADDR_PRESENT_POSITION2
            )

            (
                dxl_present_current,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read2ByteTxRx(
                portHandler, DXL_ID1, ADDR_Present_Current
            )  # dxl_Current_Limit
            # dxl_Hardware_error
            (
                dxl_Current_Limit,
                dxl_comm_result1,
                dxl_error1,
            ) = packetHandler1.read2ByteTxRx(
                portHandler, DXL_ID1, ADDR_Current_Limit
            )  # ADDR_HARDWARE_ERROR)#dxl_Current_Limit

            if dxl_present_current > 0x7FFF:
                dxl_present_current = dxl_present_current - 65536

            # print("PRO=",dxl_present_current, dxl_Current_Limit,dxl_comm_result1, dxl_error1)
            # print("xm540=",dxl_comm_result2, dxl_error2)

        if dxl_comm_result1 != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
        elif dxl_error1 != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error1))

        # if dxl_comm_result2 != COMM_SUCCESS:
        #     print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
        # elif dxl_error2 != 0:
        #     print("%s" % packetHandler2.getRxPacketError(dxl_error2))

        if dxl_comm_result3 != COMM_SUCCESS:
            print("%s" % packetHandler3.getTxRxResult(dxl_comm_result3))
        elif dxl_error3 != 0:
            print("%s" % packetHandler3.getRxPacketError(dxl_error3))
        # if St != "" :
        #     break

        if State_[St] == State_["S"]:
            St = ""
            print("--END--")
            break
        elif State_[St] == State_["VM"]:
            if (
                MY_DXL == "XL320"
            ):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                (
                    dxl_present_velocity,
                    dxl_comm_result,
                    dxl_error,
                ) = packetHandler3.read2ByteTxRx(
                    portHandler, DXL_ID3, ADDR_PRESENT_VELOCITY3
                )
            else:
                (
                    dxl_present_velocity,
                    dxl_comm_result,
                    dxl_error,
                ) = packetHandler3.read4ByteTxRx(
                    portHandler, DXL_ID3, ADDR_PRESENT_VELOCITY3
                )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler3.getRxPacketError(dxl_error))
                # print("--END--")
                # print("S-[ID:%03d] GoalPos:%03d PresentPos:%03d" % (DXL_ID, key, dxl_present_position))
                # Ang_.append("S-[ID:%03d] GoalPos:%03d PresentPos:%03d" % (DXL_ID, key, dxl_present_position))
            if abs(dxl_present_velocity) < 10:
                St = ""
                break
                # print('\n'.join(str("S==[time:%7.5f] GoalPos:%03d deg:%03d PresentPos:%03d deg:%03d "%x) for x in Ang_))
            print(dxl_present_velocity)
            # Plot(Ang_,int(A2),1)

            # else:
            #     print("[ID:%03d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID, d[int(A2)], int(A2) , dxl_present_position))
            #
            #     if not abs(d[int(A2)] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            #         break
        elif State_[St] == State_["K"] or State_[St] == State_["SM"]:
            St = ""
            print("--END--")
            break

            if key2 != "":
                key2 = float(key2)
                print(
                    "xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d"
                    % (DXL_ID2, key2, key2, dxl_present_position2)
                )

                if not abs(key2 - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD2:
                    St = ""
                    break
            if key3 != "":
                key3 = float(key3)
                print(
                    "xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d"
                    % (DXL_ID3, key3, key3, dxl_present_position3)
                )

                if not abs(key3 - dxl_present_position3) > DXL_MOVING_STATUS_THRESHOLD3:
                    St = ""
                    break

            if key1 != "":
                key1 = float(key1)
                if dxl_present_position1 < 501923:
                    print(
                        "PRO=[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d"
                        % (DXL_ID1, key1, key1, dxl_present_position1)
                    )

                    if (
                        not abs(key1 - dxl_present_position1)
                        > DXL_MOVING_STATUS_THRESHOLD
                    ):
                        St = ""
                        break
                # elif (dxl_present_position ==0 ):
                #     print("A-[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d" % (DXL_ID, int(float(key)), d[int(float(key))] , dxl_present_position-AF))

                else:
                    print(
                        "PRO-=[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d"
                        % (DXL_ID1, key1, key1, dxl_present_position1)
                    )

                    if (
                        not abs(key1 - dxl_present_position1 + AF)
                        > DXL_MOVING_STATUS_THRESHOLD
                    ):
                        St = ""
                        break

        else:
            break
            # if State_[St] == State_["K"]:
            #     key1=a2(d1,key1)
            #     key1=a2(d2,key2)
            #     print(key1,key2)

            # print("xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID2, d2[int(key2)], int(key2) , dxl_present_position2))
            #
            # if not abs(d2[int(key2)] - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD2:
            #     St=""
            #     break
            # print("xm540=[ID:%05d] GoalPos:%03d AnglePos:%03d degree PresentPos:%03d" % (DXL_ID3, d2[int(key3)], int(key3) , dxl_present_position3))
            #
            # if not abs(d2[int(key3)] - dxl_present_position3) > DXL_MOVING_STATUS_THRESHOLD3:
            #     St=""
            #     break
            #
            # if (dxl_present_position1<501923 ):
            #     print("PRO=[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d" % (DXL_ID1, int(float(key1)), d1[int(float(key1))] , dxl_present_position1))
            #
            #     if not abs(d1[int(float(key1))] - dxl_present_position1) > DXL_MOVING_STATUS_THRESHOLD:
            #         St=""
            #         break
            # # elif (dxl_present_position ==0 ):
            # #     print("A-[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d" % (DXL_ID, int(float(key)), d[int(float(key))] , dxl_present_position-AF))
            #
            # else:
            #     print("PRO-=[ID:%05d] GoalPos:%11d AnglePos:%11d degree PresentPos:%11d" % (DXL_ID1, int(float(key1)), d1[int(float(key1))] , dxl_present_position1))
            #
            #     if not abs(d1[int(float(key1))] - dxl_present_position1 + AF) > DXL_MOVING_STATUS_THRESHOLD:
            #         St=""
            #         break
        # time.sleep(0.005)
    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel Torque Pro
dxl_comm_result1, dxl_error1 = packetHandler1.write1ByteTxRx(
    portHandler, DXL_ID1, ADDR_TORQUE_ENABLE1, TORQUE_DISABLE
)
if dxl_comm_result1 != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_comm_result1))
elif dxl_error1 != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_error1))

# # Disable Dynamixel Torque xm540
# dxl_comm_result2, dxl_error2 = packetHandler2.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE2, TORQUE_DISABLE)
# if dxl_comm_result2 != COMM_SUCCESS:
#     print("%s" % packetHandler2.getTxRxResult(dxl_comm_result2))
# elif dxl_error2 != 0:
#     print("%s" % packetHandler2.getRxPacketError(dxl_error2))

# Disable Dynamixel Torque xm5402
dxl_comm_result3, dxl_error3 = packetHandler3.write1ByteTxRx(
    portHandler, DXL_ID3, ADDR_TORQUE_ENABLE1, TORQUE_DISABLE
)
if dxl_comm_result3 != COMM_SUCCESS:
    print("%s" % packetHandler3.getTxRxResult(dxl_comm_result3))
elif dxl_error3 != 0:
    print("%s" % packetHandler3.getRxPacketError(dxl_error3))

# Close port
portHandler.closePort()
# Close port
portHandlerm.closePort()
