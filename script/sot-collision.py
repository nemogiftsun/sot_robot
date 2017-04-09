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
from pinocchio_inv_dyn.robot_wrapper import RobotWrapper

from dynamic_graph_bridge_msgs.msg import Vector
from dynamic_graph_bridge_msgs.msg import Matrix





proximity_range= [0.04,0.04];
proximity_rootframe = ['elbow_joint','shoulder_pan_joint'];
N = len(proximity_range)
proximity_cell_pose = np.matlib.zeros((7,N));
'''
#idx = self.r.model.names[i+1]
fid = self.getFrameId(idx)  
frameVel = self.r.frameVelocity(fid)
frameJacobian = self.r.frameJacobian(self.q,fid,False,False)
nommass = self.r.model.inertias[i+1].mass
'''

class SotCollision:
    def __init__(self,name):
        self.urdfDir = rospack.get_path('ur_description') + '/urdf/'
        self.urdfName = 'ur5_robot.urdf'
        self.r = RobotWrapper(self.urdfDir + self.urdfName,'/opt/ros/indigo/share/', root_joint = se3.JointModelFreeFlyer());
        #self.model = se3.buildModelFromUrdf(self.urdfDir + self.urdfName, se3.JointModelFreeFlyer())
        #self.data = self.model.createData()
        self.distanceCollision = np.matlib.zeros(N);
        self.jacobianCollision = np.matlib.zeros((N,self.r.model.nv));
        rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        self.pub_JCollision = rospy.Publisher('collision_jacobian', Matrix, queue_size=10)
        self.pub_dCollision = rospy.Publisher('collision_distance', Vector, queue_size=10)
        self.count = 0
        #rospy.Subscriber("skin_sensor/proximity_range", JointTrajectoryControllerState, self.callback)
        #rospy.Subscriber("skin_sensor/proximity_root_frame", JointTrajectoryControllerState, self.callback)
        #rospy.Subscriber("skin_sensor/proximity_cell_pose", JointTrajectoryControllerState, self.callback)        
        
    def callback(self,data):
        self.q = np.matrix([0,0,0,0,0,0,1] + list(data.actual.positions))
        proximity_range[0] = self.count*0.04
        #print self.q
        self.compute_collision_constraints()
        jc = Matrix();
        jc.data = se3.utils.npToTuple(self.jacobianCollision.ravel())
        jc.width = self.jacobianCollision.shape[0];
        dc =  se3.utils.npToTuple(self.distanceCollision);
        self.pub_dCollision.publish(dc);
        self.pub_JCollision.publish(jc);
        #print self.jacobianCollision.shape
                      
    def getFrameId(self, frameName):
        if(self.r.model.existFrame(frameName)==False):
            raise NameError("[InvDynFormUtil] ERROR: frame %s does not exist!"%frameName);
        return self.r.model.getFrameId(frameName);

            
    def compute_collision_constraints(self):
        q = self.q
        self.r.forwardKinematics(q)
        for i in range(N):
            d = proximity_range[i];
            fid = self.getFrameId(proximity_rootframe[i]);
            pi = se3.utils.XYZQUATToSe3((proximity_cell_pose[:,i])) 
            pij = se3.utils.XYZQUATToSe3((0,0,d,0,0,0,1));
            pj = pi.act(pij)
            distance_normal = np.asmatrix(np.linalg.norm((pi.translation-pj.translation),axis =1)).T           
            skew_vlink = skew(pi.translation)
            frameJacobian = self.r.frameJacobian(q,fid,False,False);
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