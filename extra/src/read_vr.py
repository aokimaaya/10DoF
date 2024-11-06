#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from simple_websocket_server import WebSocketServer, WebSocket
# import rospy
# from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import time

test_without_ros = False

publisher=None

duration = 0.05


    



def send_to_robot(h1,h2,h3,r1,r2,r3,r4,r5,r6,r7,l1,l2,l3,l4,l5,l6,l7,t1,t2,t3,time=0.03):
    trajectory = create_joint_trajectoy_base(
                    publisher = publisher,
                    joint_names = [ 'head/joint_1', 'head/joint_2','head/joint_3'
                        ,'right_arm/joint_1', 'right_arm/joint_2', 'right_arm/joint_3', 'right_arm/joint_4', 'right_arm/joint_5', 'right_arm/joint_6', 'right_arm/joint_7'
                        ,'left_arm/joint_1', 'left_arm/joint_2', 'left_arm/joint_3', 'left_arm/joint_4', 'left_arm/joint_5', 'left_arm/joint_6', 'left_arm/joint_7'
                        ,'torso/joint_3'#,'torso/joint_2'#,'torso/joint_1'
                        ],
                    positions = np.deg2rad([h1,h2,h3
                                    ,r1,r2,r3,r4,r5,r6,r7
                                    ,l1,l2,l3,l4,l5,l6,l7
                                    ,t3#,t2#,t1
                                    ]),
                    time_from_start = time
                )

def create_joint_trajectoy_base(publisher, joint_names, positions, time_from_start):
#def create_joint_trajectoy_base:
    """
    Function to create an empty JointTrajectory message.

    Parameters
    ----------
    Non

    Returns
    -------
    JointTrajectroy

    Throws
    ------
    None
    """

    # Creates a message.
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0 for i in range(len(joint_names))]
    point.accelerations = [0.0 for i in range(len(joint_names))]
    point.effort = [0.0 for i in range(len(joint_names))]
    # point.time_from_start = rospy.Time(0)
    trajectory.points.append(point)
    point.time_from_start = rospy.Duration(time_from_start)

    publisher.publish(trajectory)
    return trajectory


def publish_joint_trajectory(publisher, joint_names, positions, time_from_start):
    """
    Function for publishing message to move joints

    Parameters
    ----------
    publisher : rospy.Publisher
        publisher
    joint_names : list of string
        list of joint names
    positions : list of float
        list of joint's goal positions(radian)
    time_from_start : float
        transition time from start

    Returns
    -------
    None

    Throws
    ------
    None
    """

    # Creates a message.
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * len(joint_names)
    point.accelerations = [0.0] * len(joint_names)
    point.effort = [0.0] * len(joint_names)
    point.time_from_start = rospy.Duration(time_from_start)
    trajectory.points.append(point)

    # Publish the message.
    publisher.publish(trajectory)


class AIREC_IKRig_WebSocketServer(WebSocket):
    def handle(self):
        data = np.fromstring(self.data, dtype=float, sep=',')
        print(data)
        
        if (test_without_ros == False):
            print("Sending Data To Robot")

            print(data)
            # send_to_robot(data[0] , data[1] , data[2] ,
            #     data[3] , data[4] , data[5] , data[6] , data[7] , data[8] , data[9],
            #     data[10], data[11], data[12], data[13], data[14], data[15], data[16],
            #     data[17], data[18], data[19] , duration)
            print("Sent!")
            time.sleep(duration*1.1)

        reply = "1"
        self.send_message(reply)
                

    def connected(self):
        print(self.address, 'connected')

    def handle_close(self):
        print(self.address, 'closed')

server = WebSocketServer('192.168.11.10', 8000, AIREC_IKRig_WebSocketServer)
#server = WebSocketServer('192.168.11.24', 8000, AIREC_IKRig_WebSocketServer)
print("Start running event") 
server.serve_forever()



