from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
import time
import dynamic_graph.sotcollision as sc
a = sc.SotCollision("sc")
a.createcollisionlink("base_link","box","internal",(0.65,0.65,0.25,0.0,0.0,0.175,0,0,0))
a.createcollisionlink("shoulder_right_link","box","internal",(0.20,0.23,0.65,0.0,0.0,-0.13,0,0,0))
a.createcollisionlink("shoulder_left_link","box","internal",(0.20,0.23,0.65,0.0,0.0,-0.13,0,0,0))
a.createcollisionlink("upper_left_arm","box","internal",(0.42,0.13,0.15,0.26,0.0,-0.0,0,0,0))
a.createcollisionlink("upper_right_arm","box","internal",(0.42,0.13,0.15,0.26,0.0,-0.0,0,0,0))
a.createcollisionlink("upper_left_forearm","box","internal",(0.37,0.09,0.09,0.28,0.0,-0.0,0,0,0))
a.createcollisionlink("upper_right_forearm","box","internal",(0.37,0.09,0.09,0.28,0.0,-0.0,0,0,0))
a.createcollisionlink("upper_right_wrist","box","internal",(0.1,0.09,0.05,0.1,0.0,-0.0,0,0,0))
a.createcollisionlink("upper_left_wrist","box","internal",(0.1,0.09,0.05,0.1,0.0,-0.0,0,0,0))
plug(robot.dynamic.base_joint,a.base_link)
plug(robot.dynamic.r_shoulder_pan_joint,a.shoulder_right_link)
plug(robot.dynamic.l_shoulder_pan_joint,a.shoulder_left_link)
plug(robot.dynamic.l_upper_arm_roll_joint,a.upper_left_arm)
plug(robot.dynamic.r_upper_arm_roll_joint,a.upper_right_arm)
plug(robot.dynamic.l_forearm_roll_joint,a.upper_left_forearm)
plug(robot.dynamic.r_forearm_roll_joint,a.upper_right_forearm)
plug(robot.dynamic.r_wrist_roll_joint,a.upper_right_wrist)
plug(robot.dynamic.l_wrist_roll_joint,a.upper_left_wrist)
plug(robot.dynamic.Jbase_joint,a.Jbase_link)
plug(robot.dynamic.Jr_shoulder_pan_joint,a.Jshoulder_right_link)
plug(robot.dynamic.Jl_shoulder_pan_joint,a.Jshoulder_left_link)
plug(robot.dynamic.Jl_upper_arm_roll_joint,a.Jupper_left_arm)
plug(robot.dynamic.Jr_upper_arm_roll_joint,a.Jupper_right_arm)
plug(robot.dynamic.Jl_forearm_roll_joint,a.Jupper_left_forearm)
plug(robot.dynamic.Jr_forearm_roll_joint,a.Jupper_right_forearm)
plug(robot.dynamic.Jr_wrist_roll_joint,a.Jupper_right_wrist)
plug(robot.dynamic.Jl_wrist_roll_joint,a.Jupper_left_wrist)
a.createcollisionpair("base_link","upper_left_arm")
a.createcollisionpair("base_link","upper_right_arm")
a.createcollisionpair("base_link","upper_left_forearm")
a.createcollisionpair("base_link","upper_right_forearm")
a.createcollisionpair("shoulder_left_link","upper_left_forearm")
a.createcollisionpair("shoulder_right_link","upper_right_forearm")
a.createcollisionpair("upper_left_forearm","upper_right_forearm")
a.createcollisionpair("upper_left_arm","upper_right_arm")
a.createcollisionpair("upper_left_arm","upper_right_forearm")
a.createcollisionpair("upper_left_forearm","upper_right_arm")
a.createcollisionpair("upper_left_wrist","upper_right_wrist")
a.createcollisionpair("upper_left_forearm","upper_right_wrist")
a.createcollisionpair("upper_left_arm","upper_right_wrist")
a.createcollisionpair("upper_right_wrist","upper_left_wrist")
a.createcollisionpair("upper_right_forearm","upper_left_wrist")
a.createcollisionpair("upper_right_arm","upper_left_wrist")
#collision avoidance task 
task_collision_avoidance=TaskInequality('taskcollision')
collision_feature = FeatureGeneric('collisionfeature')
plug(a.collisionJacobian,collision_feature.jacobianIN)
plug(a.collisionDistance,collision_feature.errorIN)
task_collision_avoidance.add(collision_feature.name)
task_collision_avoidance.referenceInf.value = (0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1)
task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)
task_collision_avoidance.dt.value=1
task_collision_avoidance.controlGain.value=50.0
#waisttask
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,0.0),(0,1.,0,-0.0),(0,0,1.,0.0),(0,0,0,1.),)
task_waist_metakine.feature.frame('desired')
#task_waist_metakine.feature.selec.value = '111101'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(1)
task_waist_metakine.featureDes.position.value = goal_waist
#torsotask
task_torso_metakine=MetaTaskKine6d('task_torsof_metakine',robot.dynamic,'torso_lift_joint','torso_lift_joint')
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
task_l_wrist_metakine=MetaTaskKine6d('task_l_wrist_metakine',robot.dynamic,'l_wrist_roll_joint','l_wrist_roll_joint')
#goal_l_wrist = ((1.,0,0,0.748),(0,1.,0,0.246),(0,0,1.,0.639),(0,0,0,1.),)
#goal_l_wrist = ((1.,0,0,0.649),(0,1.,0,-0.042),(0,0,1.,0.845),(0,0,0,1.),)

goal_l_wrist = ((1.,0,0,0.486),(0,1.,0,-0.251),(0,0,1.,0.826),(0,0,0,1.),)
task_l_wrist_metakine.feature.frame('desired')
task_l_wrist_metakine.feature.selec.value = '000111'#RzRyRxTzTyTx
solvertask_l_wrist_metakine.gain.setConstant(6)
task_l_wrist_metakine.featureDes.position.value = goal_l_wrist
#r_wrist_task

task_r_wrist_metakine=MetaTaskKine6d('task_r_wrist_metakine',robot.dynamic,'r_wrist_roll_joint','r_wrist_roll_joint')
#task_r_wrist_metakine=MetaTaskKine6d('task_r_wrist_metakine',robot.dynamic,'r_foream_roll_joint','r_forearm_roll_joint')
#goal_r_wrist = ((1.,0,0,0.425 ),(0,1.,0,-0.186),(0,0,1., 0.930))
goal_r_wrist = ((1.,0,0,0.455),(0,1.,0,-0.835),(0,0,1.,0.780))
#goal_r_wrist = ((1.,0,0,0.455),(0,1.,0,-0.835),(0,0,1.,0.780),(0,0,0,1.),) 
#
#goal_r_wrist = ((1.,0,0,0.455),(0,1.,0,-0.835),(0,0,1.,0.780),(0,0,0,1.),) 
task_r_wrist_metakine.feature.frame('desired')
task_r_wrist_metakine.feature.selec.value = '000111'#RzRyRxTzTyTx
task_r_wrist_metakine.gain.setConstant(6)
task_r_wrist_metakine.featureDes.position.value = goal_r_wrist

solver.damping.value =3e-2
time.sleep(1)
solver.push (taskjl.name)
time.sleep(1)
solver.push (task_collision_avoidance.name)
time.sleep(1)
solver.push (task_waist_metakine.task.name)
time.sleep(1)
solver.push (task_torso_metakine.task.name)
time.sleep(1)
solver.push (task_r_wrist_metakine.task.name)
time.sleep(1)
solver.push (task_l_wrist_metakine.task.name)
time.sleep(1)
plug (solver.control,robot.device.control)
