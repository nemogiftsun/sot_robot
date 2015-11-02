from dynamic_graph.sot.pr2.sot_interface import SOTInterface
from rqt_kws.planning_interface import KineoPlanner_Interface
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts


# Controller
sot = SOTInterface()
#########
base=[0,0,0,0,0,0]
torso=[0,0]
head = [0,0,0]
right_arm = [0.0, 0.0, 0.0, -0.1499999999999993, 0.0, -0.09999999999999967,0,0,0,0,0,0,0,0]
left_arm = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
jp = base+[torso[0]]+ head+right_arm+left_arm+ [torso[1]]

sot.setRobotPosture(jp)


#Controller Thread
@loopInThread
def inc():
    sot.robot.device.increment(0.1)


runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)
go

#Planner
from rqt_kws.planning_interface import KineoPlanner_Interface
p = KineoPlanner_Interface()
