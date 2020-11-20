#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
import numpy as np

import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Polygon, Point32, Vector3
from nav_msgs.msg import Odometry


# import vrep
import matplotlib.pyplot as plt
import sys

import time

LSignalName = "CycleLeft"
RSignalName = "CycleRight"
# BaseFreq = -3
BaseFreq = -3
coef = 3

Max_freq = BaseFreq * coef - 0.5
vel_diff = 0.01
vel_threshold = 100000

class Pursuit:
    def __init__(self, leftName, rightname):


        self.pos0 = Point32()
        self.pos1 = Point32()
        self.pos2 = Point32()
        # prepare the sub and pub
        self.sub_r_pos = rospy.Subscriber("/cockroachPos", Point32, self.getPos, queue_size=1)
        self.sub_path = rospy.Subscriber("/cockroachPath", Vector3, self.getPath, queue_size=1)
        self.pub_vel = rospy.Publisher("/cockroachVel", Vector3, queue_size = 1)
        self.pub_pathNum = rospy.Publisher("/pathNum", Vector3, queue_size = 1)


        self.LSignalName = leftName
        self.RSignalName = rightname

        self.LCycleFreq = BaseFreq
        self.RCycleFreq = BaseFreq

        self.goal_received = False
        self.goal_reached = False

        self.rPose = None

        self.last_theta = 0
        self.path_num = 0

        self.pos = Point32()     # robot position
        self.ori = None     # robot orientation

        self.path = Vector3()      # destination points

        self.kp = 2
        # self.kp = 1.5

        # self.kd = 5
        self.kd = 10

        self.ki = 0
        # self.ki = 0.01
        self.total_theta = 0

        self.eta = None

        self.radiu = 0.1     # the dist from destination point that robot stop
        self.flag = False   # check if robot reach the destination point

        # for test :: to get the handle
        self.cube = None
        self.cube1 = None

        self.last_L_freq = 0
        self.last_R_freq = 0


    def getPos(self, msg):
        """
        implement localization and getPath
        """

        self.pos = msg
        self.ori = msg.z
        

        self.controller()

    def getPath(self, msg):
        """
        get the path which need to track    TODO
        """
        # destination (for first step)
        self.path.x = msg.x
        self.path.y = msg.y



    def getEta(self, pose):     # TODO : refer to test_controller
        """
        get the eta between robot orientation and robot position to destination     TODO: how to get the delta orientation
        :param pose: tracking position
        :return: eta
        """
        vector_x = np.cos(self.ori) * (pose.x - self.pos.x) + np.sin(self.ori) * (pose.y - self.pos.y)
        vector_y = -np.sin(self.ori) * (pose.x - self.pos.x) + np.cos(self.ori) * (pose.y - self.pos.y)
        eta = math.atan2(vector_y, vector_x)
        # vector_x = (pose.x - self.pos.x)
        # vector_y = (pose.y - self.pos.y)
        # eta =math.atan(math.tan(math.atan2(vector_y, vector_x) - self.ori))
        return eta

    def if_goal_reached(self, pose):
        """
        check iff dist between robot and destination is less than limit
        :return: True / False
        """
        dx = self.pos.x - pose.x
        dy = self.pos.y - pose.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        return dist < self.radiu

    def controller(self):
        # theta = self.getEta(self.path[self.path_num])
        theta = self.getEta(self.path)
        if theta < 0:
            if theta < -1.6:
                theta = -3.14 - theta
        else:
            if theta > 1.6:
                theta = 3.14 - theta
        

        if not self.if_goal_reached(self.path):
            vel_revised = (self.kp * theta + (theta - self.last_theta) * self.kd + self.ki * self.total_theta)
            self.LCycleFreq = (BaseFreq + vel_revised)*coef
            self.RCycleFreq = (BaseFreq - vel_revised)*coef

            print("expect_L", self.LCycleFreq)
            print("expect_R", self.RCycleFreq)

            # for acceleration
            print("L : ", self.last_L_freq)
            print("R : ", self.last_R_freq)
            if abs(self.LCycleFreq - self.last_L_freq) > vel_threshold :
                print("L")
                self.LCycleFreq = self.last_L_freq - vel_diff
            if abs(self.RCycleFreq - self.last_R_freq) > vel_threshold :
                print("R")
                self.RCycleFreq = self.last_R_freq - vel_diff
        
            # for limit the vel
            if abs(self.LCycleFreq) > abs(Max_freq) :
                self.LCycleFreq = Max_freq
                self.RCycleFreq -= vel_revised
            if abs(self.RCycleFreq) > abs(Max_freq) :
                self.RCycleFreq = Max_freq
                self.LCycleFreq += vel_revised

            # if abs(self.LCycleFreq) > abs(Max_freq) and theta > 0:
            #     self.LCycleFreq = Max_freq
            #     self.RCycleFreq *= -1
            # elif abs(self.RCycleFreq) > abs(Max_freq) and theta < 0 :
            #     self.RCycleFreq = Max_freq
            #     self.LCycleFreq *= -1

            # print("goal_num : ", self.path_num)
            # print("Error : ", theta)
            # print("goal : ", self.path)
            print("self.L_vel : ", self.LCycleFreq)
            print("self.R_vel : ", self.RCycleFreq)
        else:
            print("goal reached !!")
            if self.path_num == 30:
                self.LCycleFreq = 0
                self.RCycleFreq = 0
                print("Bingo !!!")
            else:
                self.path_num += 1
                self.total_theta = 0
        self.last_theta = theta
        self.total_theta += theta

        vel = Vector3()
        vel.x = self.LCycleFreq
        vel.y = self.RCycleFreq

        pathMsg = Vector3()
        pathMsg.x = self.path_num

        self.pub_pathNum.publish(pathMsg)
        self.pub_vel.publish(vel)

        self.last_L_freq = self.LCycleFreq
        self.last_R_freq = self.RCycleFreq


if __name__=="__main__":
    rospy.init_node("cockroachRun")
    Pursuit(LSignalName, RSignalName)
    rospy.spin()
