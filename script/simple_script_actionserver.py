#!/usr/bin/env python

import rospy
import actionlib
import moveit_commander
from giftbot_action_server.msg import *

def plan_robot():
    rospy.init_node('giftbot_client_py')
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("manipulator")
    group_variable_values = [1.47,-1.48,1.95,-2.03,-1.57,0]
    # group_variable_values = [-1.15,-2.15,-1.26,-1.13,1.51,0.59]
    group.set_joint_value_target(group_variable_values)
    plan = group.plan()
    print(plan)


    client = actionlib.SimpleActionClient('giftbot_execute_server', giftbot_action_server.msg.giftbot_action_serverAction)
    client.wait_for_server()
    goal = giftbot_action_serverGoal()
    goal.goal_pose = plan
    client.send_goal(goal)
    client.wait_for_result()
    moveit_commander.os._exit(0)

if __name__=='__main__':
  try:
      plan_robot()
  except rospy.ROSInterruptException:
      pass