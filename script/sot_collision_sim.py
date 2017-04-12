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

from std_msgs.msg import ColorRGBA as Color

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
skin_cell_joint_frame = 'forerarm_skin_cell_joint_'
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

class SotCollision:
    def __init__(self,name):
        self.urdfDir = rospack.get_path('sot_robot') + '/urdf/'
        self.urdfName = 'ur5_robot_with_ring_forearm.urdf'
        self.r = RobotWrapper(self.urdfDir + self.urdfName,'/home/fiad1/ros/skin_sot_ws/src/universal_robot/', root_joint = se3.JointModelFreeFlyer());
        rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        self.pub_JCollision = rospy.Publisher('collision_jacobian', Matrix, queue_size=10)
        self.pub_dCollision = rospy.Publisher('collision_distance', Vector, queue_size=10)
        self.count = 0
        rospy.Subscriber("sot_collision_distance", Vector, self.callback_skin)
        self.marker_publisher = rospy.Publisher('sot_collision_range_fields', Marker)
        self.color_points = Color();self.color_points.r = 1;self.color_points.a=1;
        self.color_lines = Color();self.color_lines.b = 1;self.color_lines.a=1;
         
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


    def updateDistanceDirections(self):    
        points_i = Marker();points_i.type = Marker.POINTS;points_i.id = 0;
        points_j = Marker();points_j.type = Marker.POINTS;points_j.id = 1;        
        lines = Marker();lines.type = Marker.LINE_LIST;lines.id = 2;
        lines.color = self.color_lines;lines.scale.x = 0.01;
        points_i.color =self.color_points;points_i.scale.x=0.01;points_i.scale.y=0.01
        points_j.color = self.color_points;points_j.scale.x=0.01;points_j.scale.y=0.01        
        points_i.ns = points_j.ns = lines.ns = 'collision_distance_normals';
        points_i.action = points_j.action = lines.action = Marker.ADD;
        marker_frame = '/base_link'
        points_i.header.frame_id = points_j.header.frame_id = lines.header.frame_id = marker_frame       
        for i in range(self.num_cells):
            pi = Point();pj = Point();
            pi.x = self.PI[:,i][0] ;pi.y = self.PI[:,i][1] ;pi.z =self.PI[:,i][2]; 
            pj.x = self.PJ[:,i][0] ;pj.y = self.PJ[:,i][1] ;pj.z =self.PJ[:,i][2]; 
            points_i.points.append(pi);
            points_j.points.append(pj);
            lines.points.append(pi);
            lines.points.append(pj);
        self.points_i = points_i;
        self.points_j = points_j;
        self.lines = lines


    def callback_skin(self,data):
        self.num_cells = len(data.data)
        self.proximity_range  = data.data
        if self.count == 0:
            self.distanceCollision = np.matlib.zeros(self.num_cells);
            self.jacobianCollision = np.matlib.zeros((self.num_cells,self.r.model.nv));           
            self.PI = np.matlib.zeros((3,self.num_cells))
            self.PJ = np.matlib.zeros((3,self.num_cells))
            self.count+=1   

    def publishjdCollision(self):
        self.compute_collision_constraints()
        self.updateDistanceDirections()
        jc = Matrix();
        jc.data = se3.utils.npToTuple(self.jacobianCollision.ravel())
        jc.width = self.jacobianCollision.shape[0]
        dc =  se3.utils.npToTuple(self.distanceCollision)
        self.pub_dCollision.publish(dc)
        self.pub_JCollision.publish(jc)
        self.marker_publisher.publish(self.points_i)
        self.marker_publisher.publish(self.points_j)
        self.marker_publisher.publish(self.lines)
        
    def callback(self,data):
        self.q = np.matrix([0,0,0,0,0,0,1] + list(data.actual.positions))
        self.publishjdCollision()

                     
    def getFrameId(self, frameName):
        if(self.r.model.existFrame(frameName)==False):
            raise NameError("[InvDynFormUtil] ERROR: frame %s does not exist!"%frameName);
        return self.r.model.getFrameId(frameName);
            
    def compute_collision_constraints(self,ee_flag=False):
        q = self.q
        self.r.forwardKinematics(q)
        for i in range(self.num_cells):
            d = self.proximity_range[i]
            fid = self.getFrameId(skin_cell_joint_frame+str(i))
            frameJacobian = self.r.frameJacobian(q,fid,True,False)
            pi = self.r.framePosition(fid)
            pij = se3.utils.XYZQUATToSe3((0,0,d,0,0,0,1))
            pj = pi.act(pij)    
            '''
            if ee_flag == False:             
                pstar_i = frame_position.inverse() * pi  
                pstar_j = frame_position.inverse() * pj
            '''    
            self.PI[:,i] = pi.translation
            self.PJ[:,i] = pj.translation   
            distance = pi.translation-pj.translation
            distance_normal = distance/np.linalg.norm(distance);
            skew_vlink = skew(pi.translation)
            #jp = frameJacobian[0:3,:];
            #print '---------------------------'
            #print self.r.frameJacobian(q,fid,True,False)[:,6:9]
            #print '---'
            #print self.r.frameJacobian(q,fid_1,True,False)[:,6:9]   
            #print frameJacobian
            jp = frameJacobian[0:3,:]- skew_vlink*frameJacobian[3:6,:]
            self.jacobianCollision[i,:] = np.dot(distance_normal.T,jp); 
            self.distanceCollision[0,i] = d;           
             
if __name__ == '__main__':
     rospy.init_node('sot_collision')
     sc = SotCollision(rospy.get_name())
     rospy.spin()

     #JTS.run()
     #rospy.spin()
