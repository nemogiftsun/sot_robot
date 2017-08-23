import rospy
import actionlib
from giftbot_action_server.msg import *
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

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


def send_waypoints(client):  
    goal = giftbot_action_serverGoal()
    for i in range(wps.shape[0]):
        point = JointTrajectoryPoint()
        point.positions = wps[i,6:20]
        goal.goal_pose.joint_trajectory.points.append(point)
    #goal.goal_pose.joint_trajectory.points = points
    print 'Sending Goal'
    client.send_goal(goal)
    #client.wait_for_result()

if __name__=='__main__':
    rospy.init_node('giftbot_client_py')
    client = actionlib.SimpleActionClient('JTS', giftbot_action_serverAction)
    print 'Waiting for the server'
    client.wait_for_server()
    print 'Planning'
    send_waypoints(client)
