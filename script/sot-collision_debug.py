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
import math

from numpy.matlib import zeros
from pinocchio.utils import skew
from control_msgs.msg import JointTrajectoryControllerState
from dynamic_graph.sot.ur.robot_wrapper import RobotWrapper

from dynamic_graph_bridge_msgs.msg import Vector
from dynamic_graph_bridge_msgs.msg import Matrix

'''
Marker related stuffs
'''

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#from giftbot_skin_driver.msg import giftbot_sot_data


#proximity_range= [0.05,0.05];
#proximity_rootframe = ['elbow_joint','shoulder_pan_joint'];
#N = len(proximity_range)
#proximity_cell_pose = np.matlib.zeros((7,N));
'''
#idx = self.r.model.names[i+1]
fid = self.getFrameId(idx)  
frameVel = self.r.frameVelocity(fid)
frameJacobian = self.r.frameJacobian(self.q,fid,False,False)
nommass = self.r.model.inertias[i+1].mass
'''

count =0
class SotCollision:
    def __init__(self,name):
        self.urdfDir = rospack.get_path('ur_description') + '/urdf/'
        self.urdfName = 'ur5_robot.urdf'
        print rospack.get_path('ur_description')
        self.r = RobotWrapper(self.urdfDir + self.urdfName,'/home/fiad1/ros/skin_sot_ws/src/universal_robot/', root_joint = se3.JointModelFreeFlyer());
        #self.model = se3.buildModelFromUrdf(self.urdfDir + self.urdfName, se3.JointModelFreeFlyer())
        #self.data = self.model.createData()
        rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        self.pub_JCollision = rospy.Publisher('collision_jacobian', Matrix, queue_size=10)
        self.pub_dCollision = rospy.Publisher('collision_distance', Vector, queue_size=10)
        self.count = 0
        #rospy.Subscriber("giftbot/ring5_data", giftbot_sot_data, self.callback_skin)
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
        self.markerArray = MarkerArray()
        self.updateMarkerState()

    def convertPosetoVector(self,pose):
        poseVector = np.matlib.zeros(7);
        poseVector[0,0] = pose.position.x;
        poseVector[0,1] = pose.position.y;
        poseVector[0,2] = pose.position.z;
        poseVector[0,3] = pose.orientation.x;
        poseVector[0,4] = pose.orientation.y;
        poseVector[0,5] = pose.orientation.z;
        poseVector[0,6] = pose.orientation.w;
        return poseVector

    def updateMarkerState(self):

        for i in range(len(self.markerArray.markers)):
            print 'Why am I here?'
            self.markerArray.markers.pop();

        for i in range(10):
            marker = Marker()
            marker.header.frame_id = "/base_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.1
            marker.pose.position.y = 0.1
            marker.pose.position.z = 0.1+i
            self.markerArray.markers.append( marker)



    def callback_skin(self,data):
	cell_poses = data.cellPoses
        self.num_cells = len(cell_poses)
        self.distanceCollision = np.matlib.zeros(self.num_cells);
        self.jacobianCollision = np.matlib.zeros((self.num_cells,self.r.model.nv));
        self.proximity_cell_pose = np.matlib.zeros((7,self.num_cells));
        for i in range(self.num_cells):
             self.proximity_cell_pose[:,i] = self.convertPosetoVector(cell_poses[i]).T
        self.proximity_range  = data.proximities
        self.proximity_rootframe = data.rootFrames     

    def publishjdCollision(self):
        self.r.forwardKinematics(self.q)
        frameJacobian = self.r.frameJacobian(self.q,11,True,False)
        #print se3.utils.se3ToXYZQUAT(self.r.framePosition(11))
        #self.compute_collision_constraints()
        jc = Matrix();
        #jc.data = se3.utils.npToTuple(self.jacobianCollision.ravel())
        jc.data = (0.0,)*8*12
        jc.width = 8#self.jacobianCollision.shape[0]
        #dc =  se3.utils.npToTuple(self.distanceCollision)
        dc = (0.02,)*8
        self.pub_dCollision.publish(dc)
        self.pub_JCollision.publish(jc)
        self.marker_publisher.publish(self.markerArray)
        
    def callback(self,data):
        self.q = np.matrix([0,0,0,0,0,0,1] + list(data.actual.positions))
     	self.publishjdCollision()
        #print self.q
        #print self.jacobianCollision.shape
                      
    def getFrameId(self, frameName):
        if(self.r.model.existFrame(frameName)==False):
            raise NameError("[InvDynFormUtil] ERROR: frame %s does not exist!"%frameName);
        return self.r.model.getFrameId(frameName);

            
    def compute_collision_constraints(self):
        q = self.q
        self.r.forwardKinematics(q)
        for i in range(self.num_cells):
            d = self.proximity_range[i]
            fid = self.getFrameId(self.proximity_rootframe[i])
            #print self.r.framePosition(fid)
            pi = se3.utils.XYZQUATToSe3((self.proximity_cell_pose[:,i])) 
            pij = se3.utils.XYZQUATToSe3((0,0,d,0,0,0,1))
            pj = pi.act(pij)
            distance_normal = np.asmatrix(np.linalg.norm((pi.translation-pj.translation),axis =1)).T           
            skew_vlink = skew(pi.translation)
            frameJacobian = self.r.frameJacobian(q,fid,False,False)
            #print frameJacobian
            jp = frameJacobian[0:3,:]- skew_vlink*frameJacobian[3:6,:]
            self.jacobianCollision[i,:] = np.dot(distance_normal.T,jp)  
            self.distanceCollision[0,i] = d;
            
    '''        
    def frameJacobian(self, q, index, update_geometry=True, local_frame=True):
        return se3.frameJacobian(self.r.model, self.data, q, index, local_frame, update_geometry)
   
    def updateState(self):
        self.joint_positions_actual[6:12];
        self.data.forward
    '''      



               
if __name__ == '__main__':
     rospy.init_node('sot_collision')
     sc = SotCollision(rospy.get_name())
     rospy.spin()

     #JTS.run()
     #rospy.spin()
