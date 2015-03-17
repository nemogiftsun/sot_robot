from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
# tracer
robot.initializeTracer()
dimension = robot.dynamic.getDimension()
robot.device.resize (dimension)
robot.startTracer ()
robot_pose = ((1.,0,0,0.0),(0,1.,0,0.0),(0,0,1.,0),(0,0,0,1.),)
feature_waist = FeaturePosition ('waist_feature', robot.dynamic.base_joint, robot.dynamic.Jbase_joint, robot_pose)
feature_waist.selec.value = '011100'
task_waist = Task ('waist_task')
task_waist.controlGain.value = 10
task_waist.add (feature_waist.name)
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (1.006, -0.001, 0.125,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
goal = matrixToTuple(generic6dReference(prl))
feature_wrist = FeaturePosition ('wrist_feature', robot.dynamic.arm_joint_5, robot.dynamic.Jarm_joint_5, goal)
#feature_wrist.selec.value = 11100
task_wrist = Task ('wrist_task')
task_wrist.controlGain.value = 1
task_wrist.add (feature_wrist.name)
#feature_wrist.selec.value = '111'
robot.addTrace (task_wrist.name, "error")
#solver = SOT ('solver')
solver.setSize (dimension)
solver.push (task_waist.name)
solver.push (task_wrist.name)
robot.startTracer()
plug (solver.control,robot. device.control)

robot.stopTracer()
'''
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (0.506, -0.001, 0.125,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
goal = matrixToTuple(generic6dReference(zp))
'''


import matplotlib.pyplot as plt
plt.plotfile('/tmp/dg_wrist_task-error.dat', delimiter=' ', cols=(0, 1, 2),names=('iteration', 'x', 'y'), marker='.')
plt.show()


#solver.push (task_wrist.name)

robot_pose = ((1.,0,0,0.5),(0,1.,0,0),(0,0,1.,0),(0,0,0,1.),)
feature_waist = FeaturePosition ('waist_feature', robot.dynamic.base_joint, robot.dynamic.Jbase_joint, robot_pose)





# Make sure signals are recomputed even if not used in the control graph

robot.traceDefaultSignals()








############## new solver###############


dimension = robot.dynamic.getDimension()
solver = SolverKine('sot')
solver.setSize (dimension) 



# create links for collision check
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
import dynamic_graph.sotcollision as sc
#solver.damping.value =3e-2
a = sc.SotCollision("sc")
solver.damping.value =3
a.createcollisionlink("wall1","box","external",(2.04,0.015,0.3,0,0,0.0,0.0,0,0))
a.createcollisionlink("wall2","box","external",(0.015,-2.44,0.3,0,0,0,0,0,0))
a.createcollisionlink("wall3","box","external",(2.04,0.015,0.3,0,0,0.0,0.0,0,0))
a.createcollisionlink("platform1","box","external",(0.5,0.8,0.11,0.0,0.0,0.0,0,0.0,0))
a.createcollisionlink("platform2","box","external",(0.5,0.8,0.11,0.0,0.0,0.0,0,0.0,0))
a.createcollisionlink("base_link_1","box","internal",(0.1,0.4,0.11,0.26,0.0,0,0,0,0))
a.createcollisionlink("base_link_2","box","internal",(0.1,0.4,0.11,-0.26,0.0,0,0,0,0))
a.createcollisionlink("base_link_3","box","internal",(0.44,0.1,0.11,0.0,0.15,0,0,0,0))
a.createcollisionlink("base_link_4","box","internal",(0.44,0.1,0.11,0.0,-0.15,0,0,0,0))
a.wall1.value = ((1,0,0,0),(0,1,0,1.22),(0,0,1,0.12),(0,0,0,1))
a.wall2.value = ((1,0,0,1.02),(0,1,0,0),(0,0,0,0.12),(0,0,0,1))
a.wall3.value = ((1,0,0,0),(0,1,0,-1.22),(0,0,1,0.12),(0,0,0,1))
a.platform1.value = ((1,0,0,0.77),(0,1,0,0.82),(0,0,1,0.025),(0,0,0,1))
a.platform2.value = ((1,0,0,0.77),(0,1,0,-0.82),(0,0,1,0.025),(0,0,0,1))
plug(robot.dynamic.base_joint,a.base_link_1)
plug(robot.dynamic.base_joint,a.base_link_2)
plug(robot.dynamic.base_joint,a.base_link_3)
plug(robot.dynamic.base_joint,a.base_link_4)
#plug(robot.dynamic.base_joint,a.base_link)
a.Jwall1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jwall2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jwall3.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jplatform1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jplatform2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
plug(robot.dynamic.Jbase_joint,a.Jbase_link_1)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_2)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_3)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_4)
#a.createcollisionpair("base_link","wall1")
#a.createcollisionpair("base_link","wall2")
#a.createcollisionpair("base_link","wall3")
a.createcollisionpair("base_link_1","platform1")
a.createcollisionpair("base_link_1","platform2")
a.createcollisionpair("base_link_2","platform1")
a.createcollisionpair("base_link_2","platform2")
a.createcollisionpair("base_link_3","platform1")
a.createcollisionpair("base_link_3","platform2")
a.createcollisionpair("base_link_4","platform1")
a.createcollisionpair("base_link_4","platform2")
task_collision_avoidance=TaskInequality('taskcollision')
collision_feature = FeatureGeneric('collisionfeature')
plug(a.collisionJacobian,collision_feature.jacobianIN)
plug(a.collisionDistance,collision_feature.errorIN)
task_collision_avoidance.add(collision_feature.name)
task_collision_avoidance.referenceInf.value = (0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05)    
task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  
task_collision_avoidance.dt.value=1
task_collision_avoidance.controlGain.value=50.0
#task_collision_avoidance.selec.value='110001'
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,0.0),(0,1.,0,-0.7),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.gain.setConstant(8)
task_waist_metakine.featureDes.position.value = goal
task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
goal = ((1.,0,0,0.4),(0,1.,0,-0.0),(0,0,1.,0.424),(0,0,0,1.),)
task_wrist_metakine.gain.setConstant(8)
task_wrist_metakine.featureDes.position.value = goal
task_wrist_metakine.feature.selec.value='100011'
a.collisionJacobian.recompute(0)
print a.collisionJacobian
a.collisionDistance.recompute(0)
print a.collisionDistance
#solver.push (task_collision_avoidance.name)
#solver.push (task_waist_metakine.task.name)
solver.push (task_wrist_metakine.task.name)
#solver.damping.value =3e-2
plug (solver.control,robot.device.control)
######### task description for collision task   ############

        


        #self.a.collisionDistance






matrixtoTuple(generic6dReference(0.633, -0.012, 0.377,-0.025, 0.107, -1.244))

goal = ((1.,0,0,0.682),(0,1.,0,-0.026),(0,0,1.,0.516),(0,0,0,1.),)
task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
task_wrist_metakine.featureDes.position.value = goal
task_wrist_metakine.gain.setConstant(15)











# solver
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
#robot.initializeTracer()
#solver.damping.value =3e-2
#robot.startTracer()
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,-0.0),(0,1.,0,-0.0),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.feature.selec.value = '111111'
task_waist_metakine.gain.setConstant(10)
task_waist_metakine.featureDes.position.value = goal
##### jl
robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 5
taskjl.referenceInf.value = robot.dynamic.lowerJl.value
taskjl.referenceSup.value = robot.dynamic.upperJl.value
taskjl.dt.value = 1
#taskjl.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))
task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (0.506, -0.001, 0.125,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
goal = matrixToTuple(generic6dReference(prl))
print goal
solver.damping.value =3e-2
task_wrist_metakine.gain.setConstant(10)
#task_wrist_metakine.feature.selec.value = '000111'
task_wrist_metakine.featureDes.position.value = goal
solver.push (taskjl.name)
#solver.push (task_waist_metakine.task.name)
solver.push (task_wrist_metakine.task.name)
plug (solver.control,robot.device.control)

#writeGraph('/tmp/sot_control_ov)


# solver j1 

from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
#robot.initializeTracer()
#solver.damping.value =3e-2
#robot.startTracer()
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,-0.0),(0,1.,0,-0.0),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.feature.selec.value = '111111'
task_waist_metakine.gain.setConstant(10)
task_waist_metakine.featureDes.position.value = goal

#robot.dynamic.upperJl.value( 5.89921287174, 2.70526034059, 0.0, 3.57792496659, 5.84685299418, 0.0115, 0.0115)
#robot.dynamic.lowerJl.value( 0.0, 0.0, -5.18362787842, 0.0, 0.0, 0.0, 0.0)
# 0.003911720106138716, -3.1570296586735935e-05, -0.00014088555542723924, 0.0042014531849403625, 0.004239095661152881, 0.0016495573565767268, 0.0008224583251194974
##### jl
robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 10
taskjl.referenceInf.value = robot.dynamic.lowerJl.value
taskjl.referenceSup.value = robot.dynamic.upperJl.value
taskjl.dt.value = 1
#taskjl.selec.value = toFlags(range(18,25)+range(26,27)+range(28,31)+range(32,40)+range(41,42)+range(43,46)+range(47,50))

task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_1','arm_joint_1')
ip = (0.167, 0.000, 0.142,0.000, -0.000, 1.00)
goal = matrixToTuple(generic6dReference(zp))
print goal
#solver.damping.value =3e-2
task_wrist_metakine.gain.setConstant(20)
#task_wrist_metakine.feature.selec.value = '000111'
task_wrist_metakine.featureDes.position.value = goal
solver.push (taskjl.name)
#solver.push (task_waist_metakine.task.name)
solver.push (task_wrist_metakine.task.name)
plug (solver.control,robot.device.control)


#------------------------------------------------------------------------#
##############check joints without inequality ############################
import time
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
robot.initializeTracer('/home/nemogiftsun/laas/devel/data/youbot/')
############ jl task ##########################
ll = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0100692,0.0100692,-5.02655,0.0221239,0.110619,0,0)
hl = (3e10, 3e10, 3e10, 3e10, 3e10, 3e10,5.84014,2.61799,-0.015708,3.4292,5.64159,0,0)
taskjl = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskjl.position)
taskjl.controlGain.value = 10
taskjl.referenceInf.value = ll
taskjl.referenceSup.value = hl
taskjl.dt.value = 1
############ waist task ########################
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal_waist = ((1.,0,0,0.0),(0,1.,0,-0.0),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.feature.selec.value = '011100'#RzRyRxTzTyTx
task_waist_metakine.gain.setConstant(1)
task_waist_metakine.featureDes.position.value = goal_waist
############ wrist task ########################
task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.155, 0.001, 0.558 ,-0.000, 0.001, 0.010)
prl = (0.432, -0.047, 0.105,0,0,0)
#zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (0.632, -0.047, 0.105,0,0,0)
#prl = (0.606, -0.001, 0.425,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
pp = (0.567, -0.013, 0.260,0,0,0)
pr = (0.08,0.02,0.528,0,0,0)
goal = matrixToTuple(generic6dReference(ip))
solver.damping.value =3e-2
task_wrist_metakine.gain.setConstant(2)
task_wrist_metakine.feature.selec.value = '000111'
task_wrist_metakine.featureDes.position.value = goal
#robot.addTrace(task_wrist_metakine.task.name,'error')
#robot.addTrace(solver.name,'control')
solver.push (taskjl.name)
time.sleep(1)
solver.push (task_waist_metakine.task.name)
time.sleep(1)
solver.push (task_wrist_metakine.task.name)
time.sleep(1)
plug (solver.control,robot.device.control)
robot.startTracer()
#done
'''
init  = 0.0010210828602845723, -3.6894591472602656e-05, -0.0011437778191085712, 0.0016109015119463166, 0.0005323284878642198, 0.001726264508775712, 0.0008793524162665643

desired -0.0036539087865684965, 0.1554620826644334, -0.49019418771828377, 0.22030925062439108, -0.003367403518202087, 0.0, 0.0

sub = −0.002632826, 0.1554620826644334, −0.491337966, 0.221920152, −0.002835075

'''
