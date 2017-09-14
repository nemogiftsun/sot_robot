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
        #self.urdfDir = rospack.get_path('sot_robot') + '/urdf/'
        #self.urdfName = 'ur5_robot_with_ring_forearm.urdf'
        #self.r = RobotWrapper(self.urdfDir + self.urdfName,'/home/fiad1/ros/skin_sot_ws/src/universal_robot/', root_joint = se3.JointModelFreeFlyer());
        #rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        self.pub_proximity = rospy.Publisher('proximity_data', Vector, queue_size=10)
        self.pub_proximity_pose = rospy.Publisher('proximity_pose', Matrix, queue_size=10)
        self.proximity_pose = Matrix()
        self.proximity_pose.data = [0,]*56
        self.proximity_pose.width = 7
        #self.count = 0
        rospy.Subscriber("giftbot/skin_sorted_data", giftbot_sot_data, self.callback_skin)

    def callback_skin(self,data):
        self.num_cells = len(data.proximities)
        self.proximity_range = data.proximities
        self.proximity_poses_unlisted = data.cellPoses
        l = []
        for cell in self.proximity_poses_unlisted:
            lc = [cell.position.x,cell.position.y,cell.position.z,cell.orientation.x,cell.orientation.y,cell.orientation.z,cell.orientation.w]
            l.append(lc)
        self.proximity_pose.data = l
        self.pub_proximity.publish(tuple(self.proximity_range))
        self.pub_proximity_pose.publish(self.proximity_pose)  


if __name__ == '__main__':
     rospy.init_node('sot_collision')
     sc = SotCollision(rospy.get_name())
     rospy.spin()
