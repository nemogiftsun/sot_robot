import dynamic_graph.sotcollision as sc
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, FeaturePosture, Task, SOT, GainAdaptive, FeatureGeneric
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine

from dynamic_graph.ros import Ros
from dynamic_graph.entity import PyEntityFactoryClass

import time
import os
import datetime
import numpy as np


# trajectory interpolator
from dynamic_graph.sot.hpp import PathSampler 


import math
import time


import xml.etree.ElementTree as ET
devel_directory = os.environ.get('DEVEL_SRC')
file = devel_directory + '/sot_robot/src/rqt_rpc/rpc_config.xml'


#usage
#from dynamic_graph.sot.pr2.sot_interface import SOTInterface
#test = SOTInterface()

class SOTInterface:
    def __init__(self,device_type='simu'): 
        if device_type =='simu':
            self.Device=RobotSimu('Pr2')
            self.robot = Pr2('Pr2')
        else:
            self.Device=PyEntityFactoryClass('RobotDevice')  
            self.robot = Pr2(name ='Pr2',device = self.Device('Pr2_device') )
        # define robot device        
        self.dimension = self.robot.dynamic.getDimension()
        self.robot.device.resize (self.dimension)
        self.ros = Ros(self.robot)
        # define SOT solver
        self.solver = SolverKine('sot_solver')
        self.solver.setSize (self.robot.dynamic.getDimension())
        self.robot.device.resize (self.robot.dynamic.getDimension())
        # define basic tasks
        #self.joint_names = rospy.get_param('sot_controller/jrl_map') 
        self.defineBasicTasks()
        self.defineCollisionAvoidance()
        self.definePoseTask('l_wrist_roll_joint')
        self.solver.damping.value =3e-5
        self.status = 'NOT_INITIALIZED'
        self.code = [7,9,13,22,24,28,32]
        # file upload
        self.CF_tree = ET.parse(file)
        self.CF_root = self.CF_tree.getroot()
        if (self.CF_root.tag == "Trajectories"):
            self.CF_trajectories = self.CF_root.findall('trajectory')
            self.CF_count = len(self.CF_trajectories)
        # tracer
        day = str(datetime.date.today())
        t = time.strftime("-%H:%M:%S")
        dt = '/'+day+t
        self.tracerCounts = 0
        self.log_directory = os.environ.get('DEVEL_DIR') + '/logs'
        self.dt_directory = self.log_directory+dt
	if not os.path.exists(self.log_directory):
	    os.makedirs(self.log_directory)
	if not os.path.exists(self.dt_directory):
	    os.makedirs(self.dt_directory)
 
    def initializeTracer(self):
        self.tracerCounts += 1
        self.directory = self.dt_directory+'/cycle_'+ str(self.tracerCounts)
	os.mkdir(self.directory)
        self.robot.initializeTracer(self.directory) 
	#self.robot.addTrace(self.solver.name,'control')
	#self.robot.addTrace(self.robot.device.name,'control')
        #self.robot.addTrace(self.collisionAvoidance.name,'collisionDistance')
        #self.robot.addTrace(self.ros.rosSubscribe.name,'proximity')
        self.robot.addTrace(self.task_skinsensor.name,'error')
        self.robot.addTrace(self.robot.dynamic.name,'position')
        self.robot.addTrace(self.robot.dynamic.name,'l_wrist_roll_joint')
        #self.robot.addTrace(self.task_posture.name,'error')
        #self.robot.addTrace(self.task_pose_metakine.name,'error')


    def defineBasicTasks(self):
        # 1. DEFINE JOINT LIMIT TASK
        self.robot.dynamic.upperJl.recompute(0)
        self.robot.dynamic.lowerJl.recompute(0)
        ll = list(self.robot.dynamic.lowerJl.value)
        ul = list(self.robot.dynamic.upperJl.value)
        ul[13] = -2.1213
        ul[15] = -2
        ll[13] = -0.15
        ll[15] = -0.1
        self.jltaskname = self.defineJointLimitsTask(ll,ul)

        # 2. DEFINE BASE/WAIST POSITIONING TASK
        position = [0,0,0]
        self.waisttaskname = self.defineWaistPositionTask(position)
        
        # 3. DEFINE ROBOT POSTURE TASK
        self.robot.device.state.recompute(self.robot.device.state.time)
        posture = self.robot.device.state.value
        #posture[13] = 1.6
        self.posturetaskname = self.defineRobotPostureTask(posture)
    

    def changeDefaultStackToRPP(self):
        self.connectDeviceWithSolver(False)
        self.solver.clear()
        self.pushTask(self.jltaskname)
        self.pushTask(self.waisttaskname)
        self.pushTask(self.task_skinsensor.name)        
        self.task_pose_metakine.feature.selec.value = '011111'
        self.pushTask(self.posetaskname)
        self.pushTask(self.posturetaskname)
        time.sleep(1)





    def changeRPPToDefaultStack(self):
        self.connectDeviceWithSolver(False)
        self.solver.clear()
        self.pushBasicTasks()
        time.sleep(1)
        self.connectDeviceWithSolver(True)


    def pushBasicTasks(self):
        self.pushTask(self.jltaskname)
        self.pushTask(self.waisttaskname)
        self.pushTask(self.task_skinsensor.name)
        self.pushTask(self.posetaskname)
        self.pushTask(self.posturetaskname)
        #self.pushTask(self.posetaskname)
        #self.connectDeviceWithSolver(False)

    def pushTask(self,taskname):
        self.solver.push(taskname)

    def clearSolver(self):
        self.connectDeviceWithSolver(False)
        self.solver.clear()


    def connectDeviceWithSolver(self,switch=False):
        if switch == True:
            plug (self.solver.control,self.robot.device.control)
        else:
            self.robot.device.control.unplug()


    def defineJointLimitsTask(self,inf=None,sup=None):
        # joint limits
        self.taskjl=TaskInequality('taskjl')
        self.jl_feature = FeatureGeneric('jlfeature')
        self.jl_feature.jacobianIN.value = matrixToTuple(np.eye(39))
        plug(self.robot.dynamic.position,self.jl_feature.errorIN)
        self.taskjl.add(self.jl_feature.name)
        self.taskjl.referenceInf.value = (-3,-3,-3,0,0,-3.14,0.01, -2.86, -0.370000, -0.560000, -0.35,-0.65,-2.12,-3.14,-2.00,-3.14,0.0, 0.0,-0.1, -3.14,0.0, 0.0, 0.0,-0.7854, -2.140000, -0.35,-0.65,-2.12,-3.14,-2.00,-3.14,0.0, 0.0,-0.1, -3.14,0.0, 0.0, 0.0, -3.14)
        self.taskjl.referenceSup.value = (3,3,3,0,0,3.14, 0.33, 2.86,1.30,2.140000,1.30,3.75,-0.15,3.14,1.0,3.14, 0.55, 0.55,0.1, 3.14, 0.55, 0.55, 0.09,1.48353,0.56, 1.30, 3.75, -0.15, 3.14, 1.0, 3.14, 0.55, 0.55, 0.1, 3.14, 0.55, 0.55, 0.09,3.14)
        self.taskjl.dt.value = 1
        self.taskjl.dt.value=1
        return self.taskjl.name
  

    def definePoseTask(self,goal_joint):
        # joint limits
        position = (0.688,0.07,1.07)    
        self.task_pose_metakine=MetaTaskKine6d('poseTask',self.robot.dynamic,goal_joint,goal_joint)
        self.goal_pose = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)
        self.task_pose_metakine.feature.frame('desired')
        self.task_pose_metakine.feature.selec.value = '000000'#RzRyRxTzTyTx
        self.task_pose_metakine.gain.setConstant(1)
        self.task_pose_metakine.featureDes.position.value = self.goal_pose
        self.posetaskname = self.task_pose_metakine.task.name
      

    def defineWaistPositionTask(self,position):
        self.task_waist_metakine=MetaTaskKine6d('WaistTask',self.robot.dynamic,'base_joint','base_joint')
        self.goal_waist = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)
        self.task_waist_metakine.feature.frame('desired')
        self.task_waist_metakine.feature.selec.value = '011100'#RzRyRxTzTyTx
        self.task_waist_metakine.gain.setConstant(10)
        self.task_waist_metakine.featureDes.position.value = self.goal_waist
        return self.task_waist_metakine.task.name

    def setWaistPosition(self,position):
        self.goal_waist = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)

    def defineRobotPostureTask(self,posture=None):
        self.posture_feature = FeaturePosture('featurePosition')
        plug(self.robot.device.state,self.posture_feature.state)

        if (len(posture) != self.dimension):
            posture = self.robot.device.state.value
        self.posture_feature.posture.value = posture 

        postureTaskDofs = [True]*(39)
        for dof,isEnabled in enumerate(postureTaskDofs):
            if dof >= 0:
              self.posture_feature.selectDof(dof,isEnabled)

        self.task_posture=Task('Posture Task')
        self.task_posture.add(self.posture_feature.name)
        # featurePosition.selec.value = toFlags((6,24))
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 0.5
        #plug(task_posture.error,gainPosition.error)
        #plug(gainPosition.gain,task_posture.controlGain)
        self.task_posture.controlGain.value = 2.0
        return self.task_posture.name
    
    def defineCollisionAvoidance(self):
        self.collisionAvoidance = sc.SotCollision("sc")
        self.collisionAvoidance.createcollisionlink("lfaa","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfab","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfac","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfad","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfae","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfaf","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfag","box","internal",(0.25,0.09,0.09,0.2,0.0,-0.0,0,0,0))
        # hand
        self.collisionAvoidance.createcollisionlink("hand","box","external",(0.25,0.01,0.01,0.2,-1.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionpair("lfaa","hand")
        self.collisionAvoidance.createcollisionpair("lfab","hand")
        self.collisionAvoidance.createcollisionpair("lfac","hand")
        self.collisionAvoidance.createcollisionpair("lfad","hand")
        self.collisionAvoidance.createcollisionpair("lfae","hand")
        self.collisionAvoidance.createcollisionpair("lfaf","hand")
        self.collisionAvoidance.createcollisionpair("lfag","hand")
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfaa)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfab)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfac)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfad)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfae)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfaf)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.lfag)
        #jacobian plug
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfaa)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfab)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfac)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfad)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfae)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfaf)
        plug(self.robot.dynamic.JWl_forearm_roll_joint,self.collisionAvoidance.Jlfag)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.hand)
        self.collisionAvoidance.Jhand.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        plug(self.ros.rosSubscribe.proximity,self.collisionAvoidance.proximitySensor)
        self.task_skinsensor=TaskInequality('taskskinsensor')
        self.sensor_feature = FeatureGeneric('sensorfeature')
        #self.sensor_feature.jacobianIN.value = matrixToTuple( (np.zeros((7,39)) )) 
        #plug(self.robot.dynamic.Jl_forearm_roll_joint,self.sensor_feature.jacobianIN)
        plug(self.collisionAvoidance.collisionJacobian,self.sensor_feature.jacobianIN)
        plug(self.collisionAvoidance.collisionDistance,self.sensor_feature.errorIN)
        self.task_skinsensor.add(self.sensor_feature.name)
        self.task_skinsensor.referenceInf.value = (0.85,)*7
        self.task_skinsensor.referenceSup.value = (1,)*7#(0.05,)*7
        self.task_skinsensor.dt.value=0.5
        #self.task_skinsensor.controlSelec.value = '000000100000000000000000000000000000000'
        '''
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 0.15
        plug(self.task_skinsensor.error,gainPosition.error)
        plug(gainPosition.gain,self.task_skinsensor.controlGain)
        '''
        self.task_skinsensor.controlGain.value = 0.25
        #robot.addTrace()


    def setRobotPosture(self,posture):
        '''
        self.ps.resetPath()
        self.ps.setTimeStep (0.01)
        sot_config = self.robot.dynamic.position.value
        hpp_config = self._converttohpp(sot_config)
        self.ps.addWaypoint(tuple(hpp_config))
        '''
        self.posture_feature.posture.value = posture
        '''
        hpp_config = self._converttohpp(posture)  
        self.ps.addWaypoint(tuple(hpp_config))
        self.ps.configuration.recompute(self.ps.configuration.time)
        plug(self.ps.configuration,self.posture_feature.posture)         
        self.ps.start()        
        '''

        
    # robot control procedures    
    def initializeRobot(self,referenceSignal):
        self.connectDeviceWithSolver(False)
        if self.status == 'NOT_INITIALIZED':
            try:
                self.pushBasicTasks()
                self.ps = PathSampler ('ps')
                self.ps.createJointReference('l_wrist_roll_joint')
                self.ps.loadRobotModel ('sot_robot', 'planar', 'pr2_sot')                
                self.status = 'INITIALIZED'
            except ValueError:
                self.status = 'NOT_INITIALIZED'

    def startRobot(self):
        if ((self.status == 'INITIALIZED') | (self.status == 'STOPPED')) :           
            plug(self.robot.dynamic.position,self.ps.position)
            self.ps.setTimeStep (0.01)
            self.status = 'STARTED'
            self.connectDeviceWithSolver(True) 
            self.initializeTracer()
            
           
    def stopRobot(self): 
        self.connectDeviceWithSolver(False) 
        self.status = 'STOPPED'
        self.robot.stopTracer()


############ Trajectory Handler Tools#########################         

    def _converttohpp(self,point):
        temp = [0,]*44
        point = list(point)
        temp[0] = point[0]
        temp[1] = point[1]
        temp[2] = math.cos(point[5])
        temp[3] = math.sin(point[5])
        j = 4
        for i in range(33):
            if(i in self.code):
                temp[j] = math.cos(point[6+i])
                temp[j+1] = math.sin(point[6+i])      
                j = j+2 
            else: 
                temp[j] = point[6+i]
                j = j+1
        return temp

        
    def _executeTrajectory(self,destination,scenario):
        self.ps.resetPath()
        self.ps.setTimeStep (0.01)
        self.wps = ()
        for trajectory in self.CF_trajectories:
            if trajectory.attrib['name'] == 'scenario_1':
                self.wps = trajectory.findall('waypoint')
                break
        #hpp_config = self._converttohpp(self.robot.device.state.value)
        #self.ps.addWaypoint(tuple(hpp_config))
        for wp in self.wps:
            sot_config = [float(x) for x in wp.attrib['jc'].split(',')]
            hpp_config = self._converttohpp(sot_config)
            self.ps.addWaypoint(tuple(hpp_config))
            if wp.attrib['name'] == "pregrasp_configuration":
		#self.ps.addWaypoint(tuple(hpp_config))
                break

        self.connectDeviceWithSolver(False)
        time.sleep(1)
        plug(self.ps.configuration,self.posture_feature.posture)  
        #self.definePoseTask('l_wrist_roll_joint')
        #self.task_pose_metakine.featureDes.position.value = self.robot.dynamic.l_wrist_roll_joint.value
        #self.changeDefaultStackToRPP()   
        self.ps.configuration.recompute(self.ps.configuration.time)
        #time.sleep(0.1)
        self.ps.start()
        #self.task_pose_metakine.featureDes.position.value = self.robot.dynamic.l_wrist_roll_joint.value
        #self.connectDeviceWithSolver(True)
        #task_pose_metakine.featureDes.position.recompute(self.ps.configuration.time)
        #self.connectDeviceWithSolver(True)
        #self.connectDeviceWithSolver(False)
        plug(self.ps.l_wrist_roll_joint,self.task_pose_metakine.featureDes.position)
        self.task_pose_metakine.feature.selec.value = '011111'
        time.sleep(0.5)        
        self.connectDeviceWithSolver(True)
        self.robot.addTrace(self.ps.name,'configuration')
        self.robot.addTrace(self.ps.name,'l_wrist_roll_joint')
        self.robot.startTracer()        

'''        
from dynamic_graph import plug

sot.ps.resetPath()
for trajectory in sot.CF_trajectories:
    if trajectory.attrib['name'] == 'scenario_1':
        wps = trajectory.findall('waypoint')
        break

for wp in wps:
    sot_config = [float(x) for x in wp.attrib['jc'].split(',')]
    hpp_config = sot._converttohpp(sot_config)
    sot.ps.addWaypoint(tuple(hpp_config))
    if wp.attrib['name'] == "pregrasp_configuration":
        break
    
    
sot.ps.setTimeStep (0.01) 
      
sot.ps.configuration.recompute(sot.ps.configuration.time)
plug(sot.ps.configuration,sot.posture_feature.posture)         
sot.ps.start()
'''
