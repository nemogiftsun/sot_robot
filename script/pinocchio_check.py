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
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

#proximity_range= [0.05,0.05];
skin_cell_joint_frame = 'forerarm_skin_cell_joint'






urdfDir = rospack.get_path('sot_robot') + '/urdf/'
urdfName = 'ur5_robot_with_ring_forearm.urdf'
r = RobotWrapper(urdfDir + urdfName,'/home/fiad1/ros/skin_sot_ws/src/universal_robot/', root_joint = se3.JointModelFreeFlyer())