#!/usr/bin/env python
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from dynamic_graph_bridge_msgs.msg import Vector
from rospkg import RosPack
import giftbot_action_server.msg
import pinocchio as se3
import numpy as np
from dynamic_graph.sot.ur.trajectories import SmoothedNdTrajectory
from actionlib_msgs.msg import GoalStatusArray
from giftbot_action_server.msg import *
from control_msgs.msg import JointTrajectoryControllerState
from dynamic_graph.sot.hpp import PathSampler

rospack = RosPack()

wps = np.zeros((10,12))
wps[0,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,-9.433983580109384e-06, -1.570015140892183, 0.00020204444625893103, -1.5705694895909812, 0.0002106347369460476, -9.722773326359402e-05]
wps[1,:]  = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.1633176366759521, -1.5600189530879482, 0.2168396535558358, -1.621612282371775, -0.1742658433075767, -8.654260453899447e-05]
wps[2,:]  = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.3266447073354843, -1.5500227652837135, 0.43347726266541264, -1.6726550751525684, -0.3487423213520994, -7.58574758143949e-05]
wps[3,:]  = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.48997177799501646, -1.540026577479479, 0.6501148717749895, -1.723697867933362, -0.5232187993966222, -6.517234708979535e-05]
wps[4,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.6532988486545487, -1.5300303896752443, 0.8667524808845664, -1.7747406607141558, -0.6976952774411449, -5.448721836519579e-05]
wps[5,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.8166259193140809, -1.5200342018710096, 1.0833900899941433, -1.8257834534949495, -0.8721717554856678, -4.380208964059622e-05]
wps[6,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,0.979952989973613, -1.5100380140667748, 1.30002769910372, -1.876826246275743, -1.0466482335301903, -3.3116960915996675e-05]
wps[7,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,1.1432800606331452, -1.5000418262625403, 1.516665308213297, -1.9278690390565367, -1.2211247115747133, -2.2431832191397108e-05]
wps[8,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,1.3066071312926775, -1.4900456384583056, 1.7333029173228738, -1.9789118318373302, -1.3956011896192357, -1.1746703466797555e-05]
wps[9,:] = [-5e-324, 1e-323, 5e-324, 2.1967674151942275e-22, 3.457799446190768e-23, -0.7849992794352644,1.4699342019522097, -1.480049450654071, 1.9499405264324507, -2.029954624618124, -1.5700776676637587, -1.061574742197989e-06]

class JointTrajectorySequencer:
    _result = giftbot_action_server.msg.giftbot_action_serverResult()
    _goalReached = False
    def __init__(self,name):
        self.urdfDir = rospack.get_path('ur_description') + '/urdf/'
        self.urdfName = 'ur5_robot.urdf'
        self.pinocchioModel = se3.buildModelFromUrdf(self.urdfDir + self.urdfName, se3.JointModelFreeFlyer())
        self.pinocchioData = self.pinocchioModel.createData()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, giftbot_action_server.msg.giftbot_action_serverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._preempted = False
        self._as.start()
        self.pub_pc = rospy.Publisher('posture_command', Vector, queue_size=10)
        rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        #rospy.Subscriber("sot_robot/state/desired", JointTrajectoryControllerState, self.callback_desired)
        #rospy.Subscriber("/trajectory_command", JointTrajectory, self.callback_trj)
        self.rate = rospy.Rate(10) # 10hz
        self.joint_states = None
        self.ps = PathSampler ('ps')
        self.ps.loadRobotModel ('ur_description', 'anchor', 'ur5_robot')
        self.ps.setTimeStep(0.01)
        self.posture_command = None
        print 's'
              
    def callback(self,data):
        self.joint_positions_desired = data.desired.positions
        self.joint_velocities_desired = data.desired.velocities
        self.joint_positions_actual = data.desired.positions
        self.joint_velocities_actual = data.desired.velocities

    def process_trajectory(self,data,dt=0.01):
        self.ps.setTimeStep(0.01)
        self.ps.resetPath ()
        num_wps = len(data.points)
        wps = np.zeros((num_wps,12))
        time_segment = np.zeros(num_wps-1)
        for i in range(num_wps):
            wps[i,0:6] = [0.,0., 0., 0., 0.,-0.785]
            wps[i,6:] = data.points[i].positions
        for i in range(num_wps):
            ps.addWaypoint(tuple(wps_ref[i,6:12]))            
        '''    
        for j in range(num_wps-1):
            init = data.points[j].time_from_start.secs + (data.points[j].time_from_start.nsecs/1e9)
            final = data.points[j+1].time_from_start.secs + (data.points[j+1].time_from_start.nsecs/1e9)
            time_segments[j]  = (final - init)/dt
        #self.posture_traj = SmoothedNdTrajectory("posture_traj", wps_ref,0.2,3);
        #self.posture_traj = SmoothedNdTrajectory("posture_traj", wps_ref,0.2,3);
        #self.max_time
        #self.dt
        '''   
    def set_posture_command(self):
        #self.ps.position.value = (0,0,0,0,0,-0.785)+self.joint_positions_actual
        self.ps.configuration.recompute(0)
        self.posture_command = self.ps.configuration.value
       
    def run(self):
        while not rospy.is_shutdown():
            if self.posture_command != None:
                self.pub.publish(self.posture_command)
                print 'run'    
            self.rate.sleep()
        
    def goalStatusCallback(self,data):
            velocity_norm = np.linalg.norm(self.joint_velocities_actual.velocities)            
            goal_error =  np.linalg.norm(self.joint_positions_desired - self.joint_positions_actual)
            if (velocity_norm < 1e-5) and (goal_error< 1e-5):                   
                self._goalReached = True
            else:                    
                self._goalReached = False

    def execute_cb(self, goal_pose): 
        self.process_trajectory(goal_pose.joint_trajectory)
        print('R1:Goal received')
        self._preempted = False
        self._goalReached = False

        while (self._preempted == False) and (self._goalReached == False):
            self.set_posture_command()
            
        print('R2:Waited for execution')     
        if self._preempted == False:
            self._result.result = True
            print('R3:Executed') 
            self._as.set_succeeded(self._result) 
        else: 
            self._result.result = False
            print('R5:Execute stopped')       
            self._as.set_aborted(self._result)        

    def preempt_cb(self):       
        print("R4:Cancel called.")
        self._preempted = True   


if __name__ == '__main__':
     rospy.init_node('JointTrajectorySequencer', anonymous=True)
     JTS = JointTrajectorySequencer('JTS')
     JTS.run()
     rospy.spin()
     #wps_ref = wps[0:2,6:].T 
     #posture_traj = SmoothedNdTrajectory("posture_traj", wps_ref,0.2,3);
     #JTS.run()
 
 
