from dynamic_graph.sot.ur.sot_interface_tom import SOTInterface,convert_to_sot
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
import transformations as tr
from numpy.linalg import inv
import numpy as np
import math

test = SOTInterface()
test.initializeRobot()
test.startRobot()

dt = 0.01
@loopInThread
def inc():
    test.robot.device.increment(dt)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

l_arm = [0.46,-2.03,2.37,-1.33,-5.48,0.93]
r_arm = [-0.46,-1.27,-2.25,-6.15,0.45,-1.18]
l_gripper_open = [-0.79,0,-0.79,0,-0.79,0]
r_gripper_open = [-0.79,0,-0.79,0,-0.79,0]
jo  = convert_to_sot(l_arm,l_gripper_open,r_arm,r_gripper_open)
 

r_arm_center = [1.570799999999999, -2.35619, 0.0, -2.6179899999999994, 2.0943999999999994, 0.7853979999999989]
l_arm_center = [-1.5708000000000002, -0.7853979999999998, 0.0, -0.5235989999999999, -2.0943999999999994, -0.7853979999999998]

jcenter  = convert_to_sot(l_arm_center,l_gripper_open,r_arm_center,r_gripper_open)


l_gripper_close = [0,0,0,0,0,0]
r_gripper_close = [0,0,0,0,0,0]
jc  = convert_to_sot(l_arm,l_gripper_close,r_arm,r_gripper_close)



l_arm_grasp = [1.5707963249999999, -2.3561944875000003, 0.0, -2.6188316330399997, 2.0935573419599995, 0.7828848883799999]
r_arm_grasp = [-1.57205296206, -0.7853981624999999, 0.0, -0.26012387142000026, 0.9839468179800006, -0.7853981624999999]
r_arm_grasp = [-2.2870794492, -0.23122121904000004, -0.34683182856000006, 1.9980529254000006, -0.7539822359999997, -0.7853981624999999]
jgr = convert_to_sot(l_arm_grasp,l_gripper_close,r_arm_grasp,r_gripper_close)


goal_wrist = tr.euler_matrix(-2.207518516473024, -0.7323059839605381, -2.827858862808813);goal_wrist[0,3] = 0.459;goal_wrist[1,3] = 0.305;goal_wrist[2,3] = 0.496;

test.task_pose_metakine.featureDes.position.value  = goal_wrist*tr.euler_matrix(math.radians(90),math.radians(-30),math.radians(0))
'''
ros_rpycenter = [1.793, -0.119, 0.672]
sot_rpycenter = [2.091, -1.115, 0.223]

diff1 = np.array([[ 0.33356771, -0.40174373,  0.07445633,  0.        ],
       [ 0.00453398,  0.16068789, -0.72399175,  0.        ],
       [ 0.5636377 , -0.28533559,  0.04787658, -0.        ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])




l_arm_grasp = [1.5707963249999999, -2.3561944875000003, 0.0, -2.6188316330399997, 2.0935573419599995, 0.7828848883799999]
r_arm_grasp = [-1.57205296206, -0.7853981624999999, 0.0, -0.26012387142000026, 0.9839468179800006, -0.7853981624999999]
jgr = convert_to_sot(l_arm_grasp,l_gripper_close,r_arm_grasp,r_gripper_close)
ros_grasp = [-1.578, 0.041, -2.600]
#sot_grasp =        


sot_pose = np.array(test.robot.dynamic.l_wrist_2_joint)

'''
