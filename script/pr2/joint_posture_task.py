import time
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT, FeatureGeneric
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
import dynamic_graph.sotcollision as sc
from numpy import *

# JOINT LIMIT INEQUALITY 
joint_limit_task=TaskInequality('joint_inequality')
joint_feature = FeatureGeneric('joint_state_feature')
plug(robot.dynamic.position,joint_feature.errorIN)
joint_jacobian = matrixToTuple(eye(39))
joint_feature.jacobianIN.value = joint_jacobian
joint_limit_task.add(joint_feature.name)
joint_limit_task.referenceInf.value = (0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.007, -0.471238, -0.714601836603, -0.5236, -0.8, -2.3213, -1.7976931348623157e+308, -2.18, -1.7976931348623157e+308, 0.0, 0.0, -0.1, -1.7976931348623157e+308, 0.0, 0.0, 0.0, -0.7854, -2.2853981634, -0.5236, -3.9, -2.3213, -1.7976931348623157e+308, -2.18, -1.7976931348623157e+308, 0.0, 0.0, -0.1, -1.7976931348623157e+308, 0.0, 0.0, 0.0, -1.7976931348623157e+308)
joint_limit_task.referenceSup.value = (1.0, 1.0, 0.0, 0, 0.0, 3.14, 0.33, 3.007, 1.39626, 2.2853981634, 1.3963, 3.9, 0.0, 1.7976931348623157e+308, 0.0, 1.7976931348623157e+308, 0.548, 0.548, 0.1, 1.7976931348623157e+308, 0.548, 0.548, 0.09, 1.48353, 0.714601836603, 1.3963, 0.8, 0.0, 1.7976931348623157e+308, 0.0, 1.7976931348623157e+308, 0.548, 0.548, 0.1, 1.7976931348623157e+308, 0.548, 0.548, 0.088, 1.7976931348623157e+308)
joint_limit_task.dt.value=1
joint_limit_task.controlGain.value=1.0

#WAIST TASK##
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,0.0),(0,1.,0,-0.0),(0,0,1.,0.0),(0,0,0,1.),)
task_waist_metakine.feature.frame('desired')
#task_waist_metakine.feature.selec.value = '111101'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(1)
task_waist_metakine.featureDes.position.value = goal_waist

##POSTURE TASK###
posture_feature = FeaturePosture('featurePosition')
plug(robot.device.state,posture_feature.state)
robotDim = len(robot.dynamic.velocity.value)
posture_feature.posture.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
postureTaskDofs = [True]*(36)
for dof,isEnabled in enumerate(postureTaskDofs):
    if dof > 6:
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
solver.damping.value =3e-2
solver.push (joint_limit_task.name)
time.sleep(1)
solver.push (task_waist_metakine.task.name)
time.sleep(1)
solver.push (task_posture.name)
time.sleep(1)
plug (solver.control,robot.device.control)


