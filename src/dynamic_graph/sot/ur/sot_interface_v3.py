from dynamic_graph.sot.ur.robot_v3 import Ur5
from dynamic_graph.ros import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, FeaturePosture, Task, SOT, GainAdaptive, FeatureGeneric
from dynamic_graph.sot.core.matrix_util import RPYToMatrix
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
import numpy as np

from dynamic_graph.ros import Ros
from dynamic_graph.entity import PyEntityFactoryClass

# trajectory interpolator
#from dynamic_graph.sot.hpp import PathSampler 
import math
import time
import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET

file = '/home/nemogiftsun/RobotSoftware/laas/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'
#file = '/home/nemogiftsun/laasinstall/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'

#usage
'''
from dynamic_graph.sot.ur.sot_interface import SOTInterface
test = SOTInterface()
test.initializeRobot()
test.startRobot()

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    test.robot.device.increment(0.01)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)


def followTrajectoryThroughPosture(t):
    #angle_range = np.linspace(-math.pi/2.0, math.pi/2.0,100);
    angle_range = np.linspace(0, 2*math.pi);
    for idx in range(0, angle_range.shape[0]):
       angle = np.sin(angle_range[idx])
       joint_posture = (angle,) * 12
       sot.setRobotPosture(joint_posture)
       time.sleep(t)

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

'''
class SOTInterface:
    def __init__(self,device_type='simu'): 
        if device_type =='simu':
            self.robot=Ur5('Ur5')
        else:
            self.Device=PyEntityFactoryClass('RobotDevice')  
	    self.robot = Ur5('Ur5',device = self.Device('Ur_device'))
        # define robot device        
        self.dimension = self.robot.dynamic.getDimension()
	self.node_name = rospy.get_name()
	#rospy.init_node('check_life', anonymous=True,disable_signals=True)
        #rospy.init_node('publisher')
        #self.pub = rospy.Publisher('posture', String, queue_size=10)
        #self.r = rospy.Rate(10) # 10hz
        self.robot.device.resize (self.dimension)
        self.ros = Ros(self.robot)
        
        # define SOT solver
        self.solver = SolverKine('sot_solver')
        self.solver.setSize (self.robot.dynamic.getDimension())
        self.robot.device.resize (self.robot.dynamic.getDimension())
        # define basic tasks
        #self.joint_names = rospy.get_param('sot_controller/jrl_map') 
        self.defineBasicTasks()
        #self.defineCollisionAvoidance()
        self.solver.damping.value =3e-5
        self.status = 'NOT_INITIALIZED'
        self.code = [7,9,13,22,24,28,32]
        self.initializeRobot()
        self.d = 0
        '''
        # file upload
        self.CF_tree = ET.parse(file)
        self.CF_root = self.CF_tree.getroot()
        if (self.CF_root.tag == "Trajectories"):
            self.CF_trajectories = self.CF_root.findall('trajectory')
            self.CF_count = len(self.CF_trajectories)
        '''
    def callback(data):
        self.d = data
        print data

    def followTrajectoryThroughPosture(self,time):
	#angle_range = np.linspace(-math.pi/2.0, math.pi/2.0,100);
	angle_range = np.linspace(0, 2*math.pi);
	for idx in range(0, angle_range.shape[0]):
	    angle = np.sin(angle_range[idx])
	    joint_posture = (angle,) * 12
	    self.setRobotPosture(joint_posture)
	    time.sleep(time)

    def defineBasicTasks(self):
        # 1. DEFINE JOINT LIMIT TASK
        self.robot.dynamic.upperJl.recompute(0)
        self.robot.dynamic.lowerJl.recompute(0)
        ll = list(self.robot.dynamic.lowerJl.value)
        ul = list(self.robot.dynamic.upperJl.value)
        self.jltaskname = self.defineJointLimitsTask(ll,ul)

        # 2. DEFINE BASE/WAIST POSITIONING TASK
        position = [0,0,0,0,0,-0.785]
        self.waisttaskname = self.defineWaistPositionTask(position)
        
        # 3. DEFINE ROBOT POSTURE TASK
        self.robot.device.state.recompute(self.robot.device.state.time)
        posture = [0,0,0,0,0,-0.785,5.297316345615761, -4.983172114475046, 4.856191953125, -4.428600915992907, -4.505247814299959, 6.036689290915096]
        #posture[13] = 1.6
        self.posturetaskname = self.defineRobotPostureTask(posture)
    

    def changeDefaultStackToRPP(self):
        self.connectDeviceWithSolver(False)
        self.solver.clear()
        self.pushTask(self.jltaskname)
        self.pushTask(self.waisttaskname)
        self.pushTask(self.task_skinsensor.name)        
        self.pushTask(self.posturetaskname)
        #self.pushTask(self.posetaskname)




    def changeRPPToDefaultStack(self):
        self.connectDeviceWithSolver(False)
        self.solver.clear()
        self.pushBasicTasks()
        self.connectDeviceWithSolver(True)


    def pushBasicTasks(self):
        self.pushTask(self.jltaskname)
        self.pushTask(self.waisttaskname)
        #self.pushTask(self.task_skinsensor.name)
        self.pushTask(self.posturetaskname)
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
        taskjl = TaskJointLimits('Joint Limits Task')
        plug(self.robot.dynamic.position,taskjl.position)
        taskjl.controlGain.value = 5
        inf[0:6] = [-3,-3,-3,-3,-3,-3]
        sup[0:6] = [3,3,3,3,3,3]
	taskjl.referenceInf.value = inf
	taskjl.referenceSup.value = sup
        taskjl.dt.value = 1
        return taskjl.name
  

    def definePoseTask(self,goal_joint):
        # joint limits
        position = (0,0,0)    
        self.task_pose_metakine=MetaTaskKine6d('poseTask',self.robot.dynamic,goal_joint,goal_joint)
        self.goal_pose = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)
        self.task_pose_metakine.feature.frame('desired')
        #self.task_pose_metakine.feature.selec.value = '011111'#RzRyRxTzTyTx
        self.task_pose_metakine.gain.setConstant(0.3)
        self.task_pose_metakine.featureDes.position.value = self.goal_pose
        self.posetaskname = self.task_pose_metakine.task.name
      

    def defineWaistPositionTask(self,position):
        self.task_waist_metakine=MetaTaskKine6d('WaistTask',self.robot.dynamic,'root_joint','root_joint')
        self.goal_waist = RPYToMatrix( position )
        #self.goal_waist = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)
        self.task_waist_metakine.feature.frame('desired')
        #self.task_waist_metakine.feature.selec.value = '000000'#RzRyRxTzTyTx
        self.task_waist_metakine.gain.setConstant(10)
        self.task_waist_metakine.featureDes.position.value = self.goal_waist
        return self.task_waist_metakine.task.name

    def setWaistPosition(self,position):
        self.goal_waist = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)

    def defineRobotPostureTask(self,posture=None):
        self.posture_feature = FeaturePosture('featurePosition')
        plug(self.robot.device.state,self.posture_feature.state)

        # sot.posture_feature.posture.feature.value = self.robot.device.state
        if (len(posture) != self.dimension):
            posture = self.robot.device.state.value
        self.posture_feature.posture.value = self.robot.device.state.value

        postureTaskDofs = [True]*(self.robot.dimension)
        for dof,isEnabled in enumerate(postureTaskDofs):
            if dof >= 6:
              self.posture_feature.selectDof(dof,isEnabled)

        self.task_posture=Task('Posture Task')
        self.task_posture.add(self.posture_feature.name)
        # featurePosition.selec.value = toFlags((6,24))
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 1
        plug(self.task_posture.error,gainPosition.error)
        plug(gainPosition.gain,self.task_posture.controlGain)
        #self.task_posture.controlGain.value = 1
        return self.task_posture.name
    
    def defineCollisionAvoidance(self):
        self.collisionAvoidance = sc.SotCollision("sc")
        self.collisionAvoidance.createcollisionlink("lfaa","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfab","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfac","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfad","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfae","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfaf","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        self.collisionAvoidance.createcollisionlink("lfag","box","internal",(0.25,0.09,0.09,0.22,0.0,-0.0,0,0,0))
        # hand
        self.collisionAvoidance.createcollisionlink("hand","box","external",(0.25,0.01,0.01,0.22,-10,-0.0,0,0,0))
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
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfaa)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfab)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfac)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfad)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfae)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfaf)
        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.collisionAvoidance.Jlfag)
        plug(self.robot.dynamic.l_forearm_roll_joint,self.collisionAvoidance.hand)
        self.collisionAvoidance.Jhand.value = ((1,0,0,0),(0,1,0,10),(0,0,1,0),(0,0,0,1))
        plug(self.ros.rosSubscribe.proximity,self.collisionAvoidance.proximitySensor)
        self.task_skinsensor=TaskInequality('taskskinsensor')
        self.sensor_feature = FeatureGeneric('sensorfeature')
        plug(self.collisionAvoidance.collisionJacobian,self.sensor_feature.jacobianIN)
        plug(self.collisionAvoidance.collisionDistance,self.sensor_feature.errorIN)
        self.task_skinsensor.add(self.sensor_feature.name)
        self.task_skinsensor.referenceInf.value = (0.85,)*7
        self.task_skinsensor.referenceSup.value = (1.0,)*7
        self.task_skinsensor.dt.value=0.5
        self.task_skinsensor.controlSelec.value = '110001000000000000000000000000000000000'
        '''
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 0.15
        plug(self.task_skinsensor.error,gainPosition.error)
        plug(gainPosition.gain,self.task_skinsensor.controlGain)
        '''
        self.task_skinsensor.controlGain.value = 0.5


    def reWireControl(self):
        #self.stopRobot()
        self.posture_feature.posture.unplug()
        #self.startRobot()
        self.ros.rosSubscribe.add('vector','pc','posture_command')
        plug(self.ros.rosSubscribe.pc,self.posture_feature.posture)
        #self.ros.rosSubscribe.pc.recompute(self.posture_feature.posture.time)
  

    def setRobotPosture(self,posture):
        #self.ps.resetPath()
        #self.ps.setTimeStep (0.01)
        #sot_config = self.robot.dynamic.position.value
        #hpp_config = self._converttohpp(sot_config)
        #self.ps.addWaypoint(tuple(hpp_config))
        self.posture_feature.posture.value = posture
        #hpp_config = self._converttohpp(posture)  
        #self.ps.addWaypoint(tuple(hpp_config))
        #self.ps.configuration.recompute(self.ps.configuration.time)
        #plug(self.ps.configuration,self.posture_feature.posture)         
        #self.ps.start()        
    
    def initializeSkin(self):
        self.ros.rosSubscribe.add('vector','dC','collision_distance')
        self.ros.rosSubscribe.add('matrix','jC','collision_jacobian')
        
    # robot control procedures    
    def initializeRobot(self):
        self.connectDeviceWithSolver(False)
        self.initializeSkin();
        if self.status == 'NOT_INITIALIZED':            
            try:
                self.pushBasicTasks()              
                self.status = 'INITIALIZED'
            except ValueError:
                self.status = 'NOT_INITIALIZED'

    def startRobot(self):
        if ((self.status == 'INITIALIZED') | (self.status == 'STOPPED')) :   
            self.posture_feature.posture.value = self.robot.device.state.value  
            #self.robot.dynamic.position.value =     
            #plug(self.robot.dynamic.position,self.ps.position)
            #self.ps.setTimeStep (0.01)
            self.status = 'STARTED'
            time.sleep(0.1)
            self.connectDeviceWithSolver(True) 
            #self.posture_feature.posture.value = [0,0,0,0,0,0,-1.6007, -1.7271, -2.2029, -0.8079, 1.5951, -0.03099] 
           
    def stopRobot(self): 
        self.connectDeviceWithSolver(False) 
        self.status = 'STOPPED'





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

    '''   
    def _executeTrajectory(self,destination,scenario):
        self.ps.resetPath()
        self.ps.setTimeStep (0.001)
        self.wps = ()
        for trajectory in self.CF_trajectories:
            if trajectory.attrib['name'] == 'scenario_1':
                self.wps = trajectory.findall('waypoint')
                break

        for wp in self.wps:
            sot_config = [float(x) for x in wp.attrib['jc'].split(',')]
            hpp_config = self._converttohpp(sot_config)
            
            self.ps.addWaypoint(tuple(hpp_config))
            if wp.attrib['name'] == "pregrasp_configuration":
                break
        time.sleep(1)
        plug(self.ps.configuration,self.posture_feature.posture)  
        self.definePoseTask('l_wrist_roll_joint')
        self.task_pose_metakine.featureDes.position.value = self.robot.dynamic.l_wrist_roll_joint.value
        self.changeDefaultStackToRPP()   
        self.ps.configuration.recompute(self.ps.configuration.time)
        time.sleep(0.1)
        self.ps.start()
        self.connectDeviceWithSolver(True)
        plug(self.ps.l_wrist_roll_joint,self.task_pose_metakine.featureDes.position)
        #self.ps.l_wrist_roll_joint.recompute(self.ps.l_wrist_roll_joint.time)
      ''' 
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
