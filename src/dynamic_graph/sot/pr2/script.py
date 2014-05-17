import time
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
robot = Pr2('Pr2', device=RobotSimu('Pr2'))
dimension = robot.dynamic.getDimension()
robot.device.resize (dimension)
from dynamic_graph.ros import Ros
ros = Ros(robot)




#waisttask
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,0.0),(0,1.,0,-0.0),(0,0,1.,0.0),(0,0,0,1.),)
task_waist_metakine.feature.frame('desired')
#task_waist_metakine.feature.selec.value = '111101'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(10)
task_waist_metakine.featureDes.position.value = goal_waist

#torsotask
task_torso_metakine=MetaTaskKine6d('task_torso_metakine',robot.dynamic,'torso_lift_joint','torso_lift_joint')
goal_torso = ((1.,0,0,-0.05),(0,1.,0,-0.0),(0,0,1.,0.790675),(0,0,0,1.),)
task_torso_metakine.feature.frame('desired')
#task_torso_metakine.feature.selec.value = '000100'#RzRyRxTzTyTx
task_torso_metakine.gain.setConstant(10)
task_torso_metakine.featureDes.position.value = goal_torso 

# joint limits
robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 5
taskjl.referenceInf.value = robot.dynamic.lowerJl.value
taskjl.referenceSup.value = robot.dynamic.upperJl.value
taskjl.dt.value = 1
#taskjl.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))


#l_wrist_task
task_l_wrist_metakine=MetaTaskKine6d('task_l_wrist_metakine',robot.dynamic,'l_wrist_roll_joint','l_wrist_roll_joint')
goal_l_wrist = ((1.,0,0,0.748),(0,1.,0,0.246),(0,0,1.,0.639),(0,0,0,1.),)
task_l_wrist_metakine.feature.frame('desired')
task_l_wrist_metakine.feature.selec.value = '000111'#RzRyRxTzTyTx
task_l_wrist_metakine.gain.setConstant(6)
task_l_wrist_metakine.featureDes.position.value = goal_l_wrist 
#0.748, 0.246, 0.639
solver = SolverKine('sot_solver')
solver.setSize (robot.dynamic.getDimension())
robot.device.resize (robot.dynamic.getDimension())

solver.push (taskjl.name)
time.sleep(1)
solver.push (task_waist_metakine.task.name)
plug (solver.control,robot.device.control)
time.sleep(1)
solver.push (task_torso_metakine.task.name)
time.sleep(1)
#solver.push (task_l_wrist_metakine.task.name)
plug (solver.control,robot.device.control)
dt = 0.01

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    robot.device.increment(dt)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)
