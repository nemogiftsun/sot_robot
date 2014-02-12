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
task_collision_avoidance.referenceInf.value = (0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1)    
task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  
task_collision_avoidance.dt.value=1
task_collision_avoidance.controlGain.value=50.0
#task_collision_avoidance.selec.value='110001'
task_waist_metakine=MetaTaskKine6d('task_waist_metakine',robot.dynamic,'base_joint','base_joint')
goal = ((1.,0,0,0.0),(0,1.,0,0.6),(0,0,1.,0),(0,0,0,1.),)
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
