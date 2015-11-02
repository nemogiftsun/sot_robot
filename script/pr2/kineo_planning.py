import time
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, FeaturePosture, Task, SOT, GainAdaptive
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
from rqt_kws.planning_interface import KineoPlanner_Interface
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

# joint limits
robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 5
ll = list(robot.dynamic.lowerJl.value)
ul = list(robot.dynamic.upperJl.value)
ll[13] = -2.1213
ll[15] = -2
ul[13] = -0.15
ul[15] = -0.1
taskjl.referenceInf.value = ll
taskjl.referenceSup.value = ul
taskjl.dt.value = 1
#taskjl.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))

## task posture
posture_feature = FeaturePosture('featurePosition')
plug(robot.device.state,posture_feature.state)
posture_feature.posture.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#posture_feature.posture.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.19, 0.0, 0.0, 0.49, 0.10, 0.0, 0, -0.15, 0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#posture_feature.posture.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.5, 0.4, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
postureTaskDofs = [True]*(39)
for dof,isEnabled in enumerate(postureTaskDofs):
    if dof > 5:
      posture_feature.selectDof(dof,isEnabled)


#robot.features['featurePosition'].selectDof(dof,isEnabled)
task_posture=Task('robot_task_position')
task_posture.add(posture_feature.name)
# featurePosition.selec.value = toFlags((6,24))
gainPosition = GainAdaptive('gainPosition')
gainPosition.set(0.1,0.1,125e3)
gainPosition.gain.value = 5
plug(task_posture.error,gainPosition.error)
plug(gainPosition.gain,task_posture.controlGain)


solver = SolverKine('sot_solver')
solver.setSize (robot.dynamic.getDimension())
robot.device.resize (robot.dynamic.getDimension())

solver.push (taskjl.name)
time.sleep(1)
solver.push (task_waist_metakine.task.name)
time.sleep(1)
solver.push (task_posture.name)
plug (solver.control,robot.device.control)
time.sleep(1)
#solver.push (task_l_wrist_metakine.task.name)
plug (solver.control,robot.device.control)
dt = 0.1




from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    test.robot.device.increment(0.1)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

goal={'joint_name':'fl_caster_rotation_joint fl_caster_l_wheel_joint fl_caster_r_wheel_joint fr_caster_rotation_joint fr_caster_l_wheel_joint fr_caster_r_wheel_joint bl_caster_rotation_joint bl_caster_l_wheel_joint bl_caster_r_wheel_joint br_caster_rotation_joint br_caster_l_wheel_joint br_caster_r_wheel_joint torso_lift_joint head_pan_joint head_tilt_joint laser_tilt_mount_joint r_shoulder_pan_joint r_shoulder_lift_joint r_upper_arm_roll_joint r_elbow_flex_joint r_forearm_roll_joint r_wrist_flex_joint r_wrist_roll_joint r_gripper_motor_slider_joint r_gripper_motor_screw_joint r_gripper_l_finger_joint r_gripper_l_finger_tip_joint r_gripper_r_finger_joint r_gripper_r_finger_tip_joint r_gripper_joint l_shoulder_pan_joint l_shoulder_lift_joint l_upper_arm_roll_joint l_elbow_flex_joint l_forearm_roll_joint l_wrist_flex_joint l_wrist_roll_joint l_gripper_motor_slider_joint l_gripper_motor_screw_joint l_gripper_l_finger_joint l_gripper_l_finger_tip_joint l_gripper_r_finger_joint l_gripper_r_finger_tip_joint l_gripper_joint torso_lift_motor_screw_joint','dof': '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.19, 0.0, 0.0, 0.49, 0.10, 0.0, 0, -0.15, 0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0'}
Planner = KineoPlanner_Interface()
Planner.setUpGoal(goal)
