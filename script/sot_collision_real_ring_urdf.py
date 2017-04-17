# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 13:07:27 2017

@author: nemogiftsun
"""
import rospy
from rospkg import RosPack
import numpy as np
rospack = RosPack()
import pinocchio as se3

from numpy.matlib import zeros
from pinocchio.utils import skew
from control_msgs.msg import JointTrajectoryControllerState
from dynamic_graph.sot.ur.robot_wrapper import RobotWrapper
from dynamic_graph_bridge_msgs.msg import Vector
from dynamic_graph_bridge_msgs.msg import Matrix

#visualizer
from giftbot_skin_driver.msg import giftbot_sot_data
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

class SotCollision:
    def __init__(self,name):
        self.pub_proximity = rospy.Publisher('proximity_data_1', Vector, queue_size=10)
	self.r = rospy.Rate(50)
	self.proximity_range = [0.06,]*16
        rospy.Subscriber("giftbot/ring5_data", giftbot_sot_data, self.callback_skin_5)
        rospy.Subscriber("giftbot/ring4_data", giftbot_sot_data, self.callback_skin_4)

    def callback_skin_5(self,data):
        self.proximity_range[8:16] = data.proximities
	self.pub_proximity.publish(tuple(self.proximity_range))

    def callback_skin_4(self,data):
        self.proximity_range[0:8] = data.proximities
	self.pub_proximity.publish(tuple(self.proximity_range))	     
             
if __name__ == '__main__':
     rospy.init_node('sot_collision')
     sc = SotCollision(rospy.get_name())
     rospy.spin()
