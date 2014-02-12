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



# new solver
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
robot.initializeTracer()
solver.damping.value =3e-2
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,-0.0),(0,1.,0,-0.0),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.feature.selec.value = '011100'
task_waist_metakine.gain.setConstant(1)
task_waist_metakine.featureDes.position.value = goal



task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',robot.dynamic,'arm_joint_5','arm_joint_5')
ip = (0.200, -0.006, 0.377,-0.155, 0.414, -0.555)
zp = (0.198, 0.001, 0.581 ,-0.000, 0.001, 0.010)
prl = (0.506, -0.001, 0.125,-3.115, 0.228, -3.029)
aov = (-0.136, -0.005, 0.389,0.005, -0.575, 0.010)
goal = matrixToTuple(generic6dReference(zp))
print goal
task_waist_metakine.gain.setConstant(1)
#task_wrist_metakine.feature.selec.value = '111'
task_waist_metakine.featureDes.position.value = goal

robot.startTracer()
solver.push (task_wrist_metakine.task.name)
#solver.push (task_waist_metakine.task.name)
plug (solver.control,robot.device.control)

#writeGraph('/tmp/sot_control_ov)



