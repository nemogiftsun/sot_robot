from dynamic_graph.sot.ur.sot_interface_tom import *
#from dynamic_graph.sot.ur.sot_interface_tom import SOTInterface,convert_to_sot,extract_from_sot_state
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
import transformations as tr
from dynamic_graph import plug, writeGraph
import numpy as np
import math
        
r_arm_goal = [-1.46477, -0.977798, 1.62378, -1.77357, 2.25241, 0.0]

goal = [   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0   ]
goal[18:24] = r_arm_goal

test = SOTInterface()
test.initializeRobot()
test.startRobot()
dt = 0.01
@loopInThread
def inc():
    test.robot.device.increment(dt)
    test.collisionAvoidance.closestPointi.recompute(test.robot.device.state.time)
    test.collisionAvoidance.closestPointj.recompute(test.robot.device.state.time)
    if (len(test.collisionAvoidance.closestPointj.value) >> 1):
        test.line_list.points = []
        for i in range(11):
            c = ColorRGBA();c.a=1;c.b=1;
            p = Point()
            p.x = test.collisionAvoidance.closestPointi.value[0][i];
            p.y = test.collisionAvoidance.closestPointi.value[1][i];
            p.z = test.collisionAvoidance.closestPointi.value[2][i];
            test.line_list.points.append(p)
            p = Point()
            p.x = test.collisionAvoidance.closestPointj.value[0][i];
            p.y = test.collisionAvoidance.closestPointj.value[1][i];
            p.z = test.collisionAvoidance.closestPointj.value[2][i];
            test.line_list.points.append(p)
            test.line_list.color= c 
            test.line_list.header.stamp = rospy.Time.now()
    test.slp.publish(test.line_list)




runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)
