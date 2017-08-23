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

wps = np.zeros((10,20))
wps[0,:] = [0,0,0,0,0,0, 0, -1.5700, 0.0002, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.5705, 0.0002, 0]
wps[1,:]  = [0,0,0,0,0,0, 0.1633, -1.5600, 0.2168, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.6216, -0.1742, 0]
wps[2,:]  = [0,0,0,0,0,0, 0.3266, -1.5500, 0.4334,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.6726, -0.3487, 0]
wps[3,:]  = [0,0,0,0,0,0, 0.4899, -1.5400, 0.6501, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.7236, -0.5232, 0]
wps[4,:] = [0,0,0,0,0,0, 0.6532, -1.5300, 0.8667, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.7747, -0.6976, 0]
wps[5,:] = [0,0,0,0,0,0, 0.8166, -1.5200, 1.0833,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.8257, -0.8721, 0]
wps[6,:] = [0,0,0,0,0,0, 0.9799, -1.5100, 1.3000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.8768, -1.0466, 0]
wps[7,:] = [0,0,0,0,0,0, 1.1432, -1.5000, 1.5166,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.9278, -1.2211, 0]
wps[8,:] = [0,0,0,0,0,0, 1.3066, -1.4900, 1.7333,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.9789, -1.3956, 0]
wps[9,:] = [0,0,0,0,0,0, 1.4699, -1.4800, 1.9499, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-2.0299, -1.5700, 0]

'''
ps = PathSampler ('ps')
ps.loadRobotModel ('ur_description', 'anchor', 'ur5_robot')
ps.setTimeStep(0.01)
for i in range(10):
     ps.addWaypoint(tuple(wps[i,6:12])) 
ps.position.value = (0,0,0,0,0,-0.785)+(-9.433983580109384e-06, -1.570015140892183, 0.00020204444625893103, -1.5705694895909812, 0.0002106347369460476, -9.722773326359402e-05)  
'''
class JointTrajectorySequencer:
    _result = giftbot_action_server.msg.giftbot_action_serverResult()
    _goalReached = False
    def __init__(self,name):
        self.urdfDir = rospack.get_path('sot_robot') + '/urdf/'
        self.urdfName = 'ur5_robot_with_ring_forearm.urdf'
        self.pinocchioModel = se3.buildModelFromUrdf(self.urdfDir + self.urdfName, se3.JointModelFreeFlyer())
        self.pinocchioData = self.pinocchioModel.createData()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, giftbot_action_serverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._preempted = False
        self.pub_pc = rospy.Publisher('posture_command', Vector, queue_size=10)
        rospy.Subscriber("sot_robot/state", JointTrajectoryControllerState, self.callback)
        self.rate = rospy.Rate(10) # 10hz
        self.joint_states = None
        self.ps = PathSampler ('ps')
        self.ps.loadRobotModel ('sot_robot', 'anchor', 'ur5_robot_with_ring_forearm')
        self.posture_command = None
        self.joint_positions_actual = None
        self._as.start()
        self.count = 'INIT'
        self.allow_publish = True
        print 'Initialized'         
        
        
    def callback(self,data):
        self.joint_positions_desired = data.desired.positions
        self.joint_velocities_desired = data.desired.velocities
        self.joint_positions_actual = data.actual.positions
        self.joint_velocities_actual = data.actual.velocities

    def process_trajectory(self,data,dt=0.01):
        print 'processing'
        self.ps.resetPath ()
        self.ps.setTimeStep(0.01)
        self.ps.position.value = (0,0,0,0,0,0)+tuple(self.joint_positions_actual[0:3])+(0,)*8+tuple(self.joint_positions_actual[3:6])
        #self.ps.addWaypoint(tuple(self.joint_positions_actual[0:3])+(0,)*8+tuple(self.joint_positions_actual[3:6]))
        self.ps.addWaypoint(self.posture_command[6:])
        self.num_wps = len(data.points)
        print 'Number of Way Points = '+str(self.num_wps)
        self.wps = np.zeros((self.num_wps,20))
        for i in range(self.num_wps):
            self.wps[i,0:6] = [0.,0., 0., 0., 0.,0]
            self.wps[i,6:] = data.points[i].positions
        for i in range(self.num_wps):
            self.ps.addWaypoint(tuple(self.wps[i,6:20])) 
        self.ps.configuration.recompute(0)   
        self.ps.start()
        self.allow_publish = True
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
        current_state = (0,0,0,0,0,0)+tuple(self.joint_positions_actual[0:3])+(0,)*8+tuple(self.joint_positions_actual[3:6])
        self.ps.position.value = current_state; 
        self.ps.configuration.recompute(1)
        self.posture_command = self.ps.configuration.value
        if self.count == 'NOT_INIT':
            self.goalStatusCallback()
        self.rate.sleep()
        velocity_norm = np.linalg.norm(self.joint_velocities_actual)
        print self.posture_command
        if (velocity_norm > 0) and (self.count == 'INIT'):
            print 'Initial State passed'
            self.count = 'NOT_INIT'
        
       
    def run(self):
        while not rospy.is_shutdown():
            if self.joint_positions_actual != None and self.posture_command == None:
                current_state = (0,0,0,0,0,0)+tuple(self.joint_positions_actual[0:3])+(0,)*8+tuple(self.joint_positions_actual[3:6])
                self.posture_command = current_state
            if self.posture_command != None and self.allow_publish:                
                self.pub_pc.publish(self.posture_command)   
            self.rate.sleep()
        
    def goalStatusCallback(self):
            print 'feedback'
            velocity_norm = np.linalg.norm(self.joint_velocities_actual)   
            print velocity_norm
            wps_final = np.hstack((self.wps[self.num_wps-1,6:9],self.wps[self.num_wps-1,17:20])) 
            print wps_final
            print np.array(self.joint_positions_actual)
            #print wps_final - np.array(self.joint_positions_actual)
            goal_error =  np.linalg.norm(wps_final - np.array(self.joint_positions_actual))
            print goal_error
            if (velocity_norm < 1e-5) and (goal_error< 1e-3):                   
                self._goalReached = True
            else:                    
                self._goalReached = False

    def execute_cb(self, goal): 
        print 'trying...'
        self.allow_publish = False
        self.process_trajectory(goal.goal_pose.joint_trajectory)
        print('R1:Goal received')
        self._preempted = False
        self._goalReached = False
        self.count = 'INIT'
    
        while (self._preempted == False) and (self._goalReached == False):
            print 's'
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
    try:
        rospy.init_node('JTS')
        JTS = JointTrajectorySequencer(rospy.get_name())
        JTS.run()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

     #rospy.spin()
     #wps_ref = wps[0:2,6:].T 
     #posture_traj = SmoothedNdTrajectory("posture_traj", wps_ref,0.2,3);
     #JTS.run()
 
 
