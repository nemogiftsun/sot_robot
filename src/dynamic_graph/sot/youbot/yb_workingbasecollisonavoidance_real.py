from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
import dynamic_graph.sotcollision as sc
#solver.damping.value =3e-2
a = sc.SotCollision("sc")
#solver.damping.value =3
a.createcollisionlink("base_link_1","box","internal",(0.1,0.4,0.11,0.26,0.0,0,0,0,0))
a.createcollisionlink("base_link_2","box","internal",(0.1,0.4,0.11,-0.26,0.0,0,0,0,0))
a.createcollisionlink("base_link_3","box","internal",(0.44,0.1,0.11,0.0,0.15,0,0,0,0))
a.createcollisionlink("base_link_4","box","internal",(0.44,0.1,0.11,0.0,-0.15,0,0,0,0))
a.createcollisionlink("wall1","box","external",(0.015,-0.7,0.3,0,0,0,0,0,0))
a.createcollisionlink("wall2","box","external",(0.015,-0.7,0.3,0,0,0,0,0,0))
a.createcollisionlink("platform1","box","external",(0.6,-1.03,0.1,0,0,0,0,0,0))
a.createcollisionlink("platform2","box","external",(2.6,0.4,0.11,0.0,0.0,0.0,0,0.0,0))
a.createcollisionlink("platform3","box","external",(1.0,0.5,0.11,0.0,0.0,0.0,0,0.0,0))
a.createcollisionlink("platform4","box","external",(0.8,0.5,0.11,0.0,0.0,0.0,0,0.0,0))
a.platform1.value = ((1,0,0,1.30),(0,1,0,0.0),(0,0,0,0.0),(0,0,0,1))
a.platform2.value = ((1,0,0,1.70),(0,1,0,-1.8),(0,0,1,0.025),(0,0,0,1))
a.platform3.value = ((1,0,0,2.55),(0,1,0,1.45),(0,0,1,0.025),(0,0,0,1))
a.platform4.value = ((1,0,0,0.85),(0,1,0,1.45),(0,0,1,0.025),(0,0,0,1))
a.wall1.value = ((1,0,0,0.4),(0,1,0,1.3),(0,0,1,0.025),(0,0,0,1))
a.wall2.value = ((1,0,0,0.4),(0,1,0,-1.37),(0,0,1,0.025),(0,0,0,1))
plug(robot.dynamic.base_joint,a.base_link_1)
plug(robot.dynamic.base_joint,a.base_link_2)
plug(robot.dynamic.base_joint,a.base_link_3)
plug(robot.dynamic.base_joint,a.base_link_4)
#plug(robot.dynamic.base_joint,a.base_link)
a.Jplatform1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jplatform2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jplatform3.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jplatform4.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jwall1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
a.Jwall2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
plug(robot.dynamic.Jbase_joint,a.Jbase_link_1)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_2)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_3)
plug(robot.dynamic.Jbase_joint,a.Jbase_link_4)
#a.createcollisionpair("base_link","wall1")
#a.createcollisionpair("base_link","wall2")
#a.createcollisionpair("base_link","wall3")
a.createcollisionpair("base_link_1","platform1")
a.createcollisionpair("base_link_1","platform2")
a.createcollisionpair("base_link_1","platform3")
a.createcollisionpair("base_link_1","platform4")
a.createcollisionpair("base_link_1","wall1")
a.createcollisionpair("base_link_1","wall2")

a.createcollisionpair("base_link_2","platform1")
a.createcollisionpair("base_link_2","platform2")
a.createcollisionpair("base_link_2","platform3")
a.createcollisionpair("base_link_2","platform4")
a.createcollisionpair("base_link_2","wall1")
a.createcollisionpair("base_link_2","wall2")

a.createcollisionpair("base_link_3","platform1")
a.createcollisionpair("base_link_3","platform2")
a.createcollisionpair("base_link_3","platform3")
a.createcollisionpair("base_link_3","platform4")
a.createcollisionpair("base_link_3","wall1")
a.createcollisionpair("base_link_3","wall2")

a.createcollisionpair("base_link_4","platform1")
a.createcollisionpair("base_link_4","platform2")
a.createcollisionpair("base_link_4","platform3")
a.createcollisionpair("base_link_4","platform4")
a.createcollisionpair("base_link_4","wall1")
a.createcollisionpair("base_link_4","wall2")
task_collision_avoidance=TaskInequality('taskcollision')
collision_feature = FeatureGeneric('collisionfeature')
plug(a.collisionJacobian,collision_feature.jacobianIN)
plug(a.collisionDistance,collision_feature.errorIN)
task_collision_avoidance.add(collision_feature.name)
task_collision_avoidance.referenceInf.value = (0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1)    
task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  
task_collision_avoidance.dt.value=1
task_collision_avoidance.controlGain.value=50.0
#task_collision_avoidance.selec.value='110001'
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,0.6),(0,1.,0,0.0),(0,0,1.,0),(0,0,0,1.),)
task_waist_metakine.gain.setConstant(8)
task_waist_metakine.featureDes.position.value = goal
a.collisionJacobian.recompute(0)
print a.collisionJacobian
a.collisionDistance.recompute(0)
print a.collisionDistance
solver.push (task_collision_avoidance.name)
solver.push (task_waist_metakine.task.name)
#solver.damping.value =3e-2
plug (solver.control,robot.device.control)