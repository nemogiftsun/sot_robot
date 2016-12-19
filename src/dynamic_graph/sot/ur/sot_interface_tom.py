from dynamic_graph.sot.ur.tom import Tom
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, FeaturePosture, Task, SOT, GainAdaptive, FeatureGeneric
from dynamic_graph.sot.core.matrix_util import RPYToMatrix
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
import rospy


from dynamic_graph.ros import Ros
from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.core.matrix_util import *
import dynamic_graph.sotcollision as sc

# trajectory interpolator
#from dynamic_graph.sot.hpp import PathSampler 


# visualizer
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import math
import time


import xml.etree.ElementTree as ET
file = '/home/nemogiftsun/RobotSoftware/laas/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'
#file = '/home/nemogiftsun/laasinstall/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'

#usage

'''
from dynamic_graph.sot.ur.sot_interface_tom import *
#from dynamic_graph.sot.ur.sot_interface_tom import SOTInterface,convert_to_sot,extract_from_sot_state
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
import transformations as tr
from dynamic_graph import plug, writeGraph
import numpy as np
import math

test = SOTInterface()
test.initializeRobot()
test.startRobot()
dt = 0.01
@loopInThread
def inc():
    test.robot.device.increment(dt)
    test.collisionAvoidance.closestPointi.recompute(test.robot.device.state.time)
    test.collisionAvoidance.closestPointj.recompute(test.robot.device.state.time)
    if (len(test.collisionAvoidance.closestPointj.value) >> 1):
        test.line_list.points = []
        for i in range(11):
            c = ColorRGBA();c.a=1;c.g=1;
    	        if test.collisionAvoidance.collisionDistance.value[i] <= 0.05:
                 c.g=0
   	    else: 
             c.g=1
             c.r=0
    	    p = Point()
    	    p.x = test.collisionAvoidance.closestPointi.value[0][i];
    	    p.y = test.collisionAvoidance.closestPointi.value[1][i];
    	    p.z = test.collisionAvoidance.closestPointi.value[2][i];
    	    test.line_list.points.append(p)
    	    p = Point()
    	    p.x = test.collisionAvoidance.closestPointj.value[0][i];
    	    p.y = test.collisionAvoidance.closestPointj.value[1][i];
    	    p.z = test.collisionAvoidance.closestPointj.value[2][i];
    	    test.line_list.points.append(p)
    	    test.line_list.colors.append(c)          
    	    test.line_list.header.stamp = rospy.Time.now()     
    test.slp.publish(test.line_list)



runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)
'''

########## MOTION PLANNING #####################

'''
from dynamic_graph.sot.hpp import PathSampler
ps = PathSampler ('ps')
ps.loadRobotModel ('tom_description', 'anchor', 'tom_hpp')
ps.setTimeStep(0.01)
plug(sot.robot.device.state,ps.position)
q_init = 12*[0]
q_end = 12*[0]
q_end[0] = 0.5;q_end[5]=0.2
final = 12*[0]
ps.addWaypoint (tuple (q_init))
ps.addWaypoint (tuple (q_end))
plug(ps.configuration,sot.posture_feature.posture)
ps.start()


def send_planrequest(r,planning_group):
	r = _moveit_robot_interface.RobotInterface("robot_description")  
	jointList = r.get_group_joint_names(userdata.planning_group) 
	# set up start state
	startState = RobotState()
	startState.deserialize(r.get_current_state())
	# Get the number of active joints
	goalConstraints = set_constraints(r,jointList,userdata.goal)
	# set up request
	resp = callPathPlanService(planning_group, startState, goalConstraints,goal)  

def callPathPlanService(planningGroup, startState, goalConstraints,goal):                           
    req = MotionPlanRequest()
    req.allowed_planning_time = 5;
    req.group_name = planningGroup
    req.start_state = startState
    req.goal_constraints = [goalConstraints]       
    get_motion_plan = rospy.ServiceProxy('/kws_ros_planner/plan_kinematic_path', GetMotionPlan)      
    try:
        res = get_motion_plan(req) 
    except rospy.ServiceException, e:
        rospy.logerr("Service did not process request: " + str(e))     
        return -1         
               
    jnames = res.motion_plan_response.trajectory.joint_trajectory.joint_names   
    jpoints =  res.motion_plan_response.trajectory.joint_trajectory.points
    return [jnames, jpoints] 

def set_constraints(r,jointList,goal):
        goalConstraints = Constraints()
        activeJointCount = 0
        jointConstraintsSize = 0
        for joint in jointList:
            jointLimits = r.get_joint_limits(str(joint))
            if len(jointLimits) > 0:
                activeJointCount = activeJointCount + 1
                jointConstraintsSize = jointConstraintsSize + len(jointLimits) 
        goalConstraints.joint_constraints = [None] * jointConstraintsSize        
        #m_baseGoal = [None] * 3
        m_jtPosition = [None] * activeJointCount          
        # Since jointList can contain some fixed joint (i.e. not active)
        # we recreate a list with only active joints
        m_activeJointList = [None] * activeJointCount
        i = 0
        for joint in jointList:
            jointLimits = r.get_joint_limits(str(joint))
            if len(jointLimits) > 0:
                m_activeJointList[i] = str(joint)
                i = i + 1
        i = 0
        resIndex = 0;
        for joint in m_activeJointList:
            #jointLimits = r.get_joint_limits(str(joint)) 
            jointConstraint = JointConstraint()
            jointConstraint.position = goal[i]   
            jointConstraint.joint_name = joint       
            goalConstraints.joint_constraints[i] = jointConstraint
            i = i + 1   
        return goalConstraints

'''

def make_markers(mid,marker_type,scale):
    m = Marker()
    if marker_type == 'POINTS':
    	m.type = m.POINTS
    elif marker_type == 'LINE_STRIP':
        m.type = m.LINE_STRIP
    elif marker_type == 'LINE_LIST':
        m.type = m.LINE_LIST
    m.header.frame_id = "/world"
    m.header.stamp = rospy.Time.now()
    m.action = m.ADD
    m.id = mid
    m.ns = "points_and_lines";
    m.scale.x = scale[0]
    m.scale.y = scale[1]
    m.scale.z = scale[2]
    m.pose.orientation.w = 1.0
    m.color.r = 0
    m.color.g = 0
    m.color.b = 1
    m.color.a = 1
    return m





class skin_line_publisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher('skin_sensor_range', Marker,queue_size=10)
        self.rate = rospy.Rate(10) # 10hz       
    def publish(self,points):
	self.marker_pub.publish(points);  
	#self.marker_pub.publish(line_strip);
	#self.marker_pub.publish(line_list);

def convert_to_sot(r_arm,r_gripper,l_arm,l_gripper):
    wheels = [0,0,0,0];
    base = [0,0,0,0,0,0]
    jp = base+ l_arm + l_gripper + r_arm + r_gripper + wheels
    return jp

def extract_from_sot_state(group,js):
    if group == 'l_arm':
        return js[6:12]
    if group == 'r_arm':
        return js[18:24]
    if group == 'l_gripper':
        return js[12:18]
    if group == 'r_gripper':
        return js[24:30]
    if group == 'base':      
        return js[0:6]

class SOTInterface:
    def __init__(self,device_type='simu'): 
        if device_type =='simu':
            self.robot=Tom('Tom')
        else:
            self.Device=PyEntityFactoryClass('RobotDevice')  
	    self.robot = Tom('Tom',device = self.Device('Ur_device'))
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
        self.ros.rosSubscribe.add('vector','proximity','proximity_data')
        self.defineBasicTasks()
        #self.defineCollisionAvoidanceDeliv()
        self.solver.damping.value =7e-3
        self.status = 'NOT_INITIALIZED'
        self.code = [7,9,13,22,24,28,32]
	self.initializeRobot()
        rospy.init_node('skin_line_publisher', anonymous=True)
        self.slp = skin_line_publisher()
	self.points = make_markers(0,'POINTS',[0.01,0.01,0.01])
	#self.line_strip = make_markers( 'LINE_STRIP',[0.05,0.05,0.05])
	self.line_list  = make_markers(1,'LINE_LIST' ,[0.01,0.01,0.01])
        #self.slp.publish(self.points)
        #self.slp.publish(self.line_list)
        '''
        # file upload
        self.CF_tree = ET.parse(file)
        self.CF_root = self.CF_tree.getroot()
        if (self.CF_root.tag == "Trajectories"):
            self.CF_trajectories = self.CF_root.findall('trajectory')
            self.CF_count = len(self.CF_trajectories)
        '''

    def defineBasicTasks(self):
        # 1. DEFINE JOINT LIMIT TASK
        self.robot.dynamic.upperJl.recompute(0)
        self.robot.dynamic.lowerJl.recompute(0)
        ll = list(self.robot.dynamic.lowerJl.value)
        ul = list(self.robot.dynamic.upperJl.value)
        #ll[2] = 0.5; ul[2]=0.5; 
        self.jltaskname = self.defineJointLimitsTask(ll,ul)


       # SAFETY TASK
        #self.defineCollisionAvoidanceDeliv()

        # 2. DEFINE BASE/WAIST POSITIONING TASK
        position = [0,0,0.0,2.606, -0.627, -0.535]
        self.waisttaskname = self.defineWaistPositionTask(position)
        
        # 3. DEFINE ROBOT POSTURE TASK
        self.robot.device.state.recompute(self.robot.device.state.time)
        posture = self.robot.device.state.value
        #posture[13] = 1.6

        self.posturetaskname = self.defineRobotPostureTask(posture)
        self.goal_pose_wrist = RPYToMatrix((0.808, 0.301, 0.960,-2.1493607154807215, -0.7416515342006911, 1.237964870888867))
        self.skinrootpose = RPYToMatrix(( 0.5745, -0.57713,1.62168,2.105, 0.626, 2.606))
        #self.lwristposetask =  self.definePoseTask('l_wrist_2_joint',self.goal_pose_wrist)
        #self.skinroot =  self.definePoseTask('r_forerarm_skin_root_cell_joint',self.skinrootpose)

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
        #self.pushTask(self.skinroot)
        #self.pushTask(self.waisttaskname)
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
        inf[0:6] = [0,0,0,0,0,0]
        sup[0:6] = [0,0,0,0,0,0]
	taskjl.referenceInf.value = inf
	taskjl.referenceSup.value = sup
        taskjl.dt.value = 1
        return taskjl.name
  

    def definePoseTask(self,goal_joint,goal_pose):
        # joint limits   
        self.task_pose_metakine=MetaTaskKine6d('skinposeTask',self.robot.dynamic,goal_joint,goal_joint)
        #self.goal_pose = ((1.,0,0,position[0]),(0,1.,0,position[1]),(0,0,1.,position[2]),(0,0,0,1.),)
        self.task_pose_metakine.feature.frame('desired')
        self.task_pose_metakine.feature.selec.value = '111000'#RzRyRxTzTyTx
        self.task_pose_metakine.gain.setConstant(1)
        self.task_pose_metakine.featureDes.position.value = goal_pose
        self.posetaskname = self.task_pose_metakine.task.name
        return self.posetaskname
      

    def defineWaistPositionTask(self,position):
        self.task_waist_metakine=MetaTaskKine6d('WaistTask',self.robot.dynamic,'base_joint','base_joint')
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

        #if (len(posture) != self.dimension):
        posture = self.robot.device.state.value
        posture_init= (0.0,0.0,0.0,0.0,0.0,0.0,0.46,-2.03,2.37,-1.33,-5.48,0.93,-0.46,-1.27,-2.25,-6.15,0.45,-1.18)
        self.posture_feature.posture.value = posture 

        postureTaskDofs = [True]*(self.robot.dimension)
        for dof,isEnabled in enumerate(postureTaskDofs):
            if dof >= 6:
              self.posture_feature.selectDof(dof,isEnabled)

        self.task_posture=Task('Posture Task')
        self.task_posture.add(self.posture_feature.name)
        # featurePosition.selec.value = toFlags((6,24))
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 0.2
        #plug(task_posture.error,gainPosition.error)
        #plug(gainPosition.gain,task_posture.controlGain)
        self.task_posture.controlGain.value = 1.0
        return self.task_posture.name
    '''
    r_forerarm_skin_link_101 <origin xyz="0.122545 0.0452926 -0.0396807" rpy="0.122545 0.0452926 -0.0396807."/>
    r_forerarm_skin_link_111 <origin xyz="0.122971 0.0347329 -0.0140614" rpy="5.35  0.07 -0."/
    r_forerarm_skin_link_83 <origin xyz="0.120033 0.0253832 -0.088699" rpy="3.67  0.07 -0."/>
    r_forerarm_skin_link_87  <origin xyz="0.122276 0.0426263 -0.0664088" rpy="4.23  0.07 -0."/>
    r_forerarm_skin_link_14  <origin xyz="0.121427 0.0122404 -0.00364349" rpy="0.25  0.05  0.""
    r_forerarm_skin_link_28  <origin xyz="0.120852 -0.0153018 -0.00418709" rpy="0.3   0.05  0."/>
    r_forerarm_skin_link_30  <origin xyz="0.119741 -0.0385305 -0.0190389" rpy="0.83 -0.04 -0."/>
    r_forerarm_skin_link_47  <origin xyz="0.119756 -0.0512256 -0.0430332" rpy="1.52  0.   -0."/>
    r_forerarm_skin_link_59 <origin xyz="0.118712 -0.0446355 -0.0687232" rpy="2.02  0.    0."/>
    r_forerarm_skin_link_62  <origin xyz="0.119505 -0.0267679 -0.0889594" rpy="2.55 -0.02 -0."/>
    r_forerarm_skin_link_82  <origin xyz="0.119639 -0.00112243 -0.0964335" rpy="3.11  0.07 -0."/>
    '''

    '''
    r_forerarm_skin_link_101 <origin xyz="0.122545,0.0452926,-0.0396807,0.122545 0.0452926 -0.0396807
    r_forerarm_skin_link_111 <origin xyz="0.122971,0.0347329,-0.0140614,5.35,0.07,-0
    r_forerarm_skin_link_83 <origin xyz="0.120033,0.0253832,-0.088699,3.67,0.07,-0
    r_forerarm_skin_link_87  <origin xyz="0.122276,0.0426263,-0.0664088,4.23,0.07,-0
    r_forerarm_skin_link_14  <origin xyz="0.121427,0.0122404,-0.00364349,0.25,0.05 ,0
    r_forerarm_skin_link_28  <origin xyz="0.120852,-0.0153018,-0.00418709,0.3, 0.05 ,0
    r_forerarm_skin_link_30  <origin xyz="0.119741,-0.0385305,-0.0190389,0.83,-0.04,-0
    r_forerarm_skin_link_47  <origin xyz="0.119756,-0.0512256,-0.0430332,1.52, 0. ,-0
    r_forerarm_skin_link_59 <origin xyz="0.118712,-0.0446355,-0.0687232,2.02 ,0.,0
    r_forerarm_skin_link_62  <origin xyz="0.119505,-0.0267679,-0.0889594,2.55,-0.02,-0
    r_forerarm_skin_link_82  <origin xyz="0.119639,-0.00112243,-0.0964335,3.11,0.07,-0
    '''    
    def defineCollisionAvoidanceDeliv(self):
        self.collisionAvoidance = sc.SotCollision("sc")
        '''
        self.collisionAvoidance.createcollisionlink("skin_101","box","internal",(0.25,0.09,0.09,0.122545,0.0452926,-0.0396807,0.122545,0.0452926,-0.0396807))
        self.collisionAvoidance.createcollisionlink("skin_111","box","internal",(0.25,0.09,0.09,0.122971,0.0347329,-0.0140614,5.35,0.07,-0))
        self.collisionAvoidance.createcollisionlink("skin_83","box","internal",(0.25,0.09,0.09,0.120033,0.0253832,-0.088699,3.67,0.07,-0))
        self.collisionAvoidance.createcollisionlink("skin_87","box","internal",(0.25,0.09,0.09,0.122276,0.0426263,-0.0664088,4.23,0.07,-0))
        self.collisionAvoidance.createcollisionlink("skin_14","box","internal",(0.25,0.09,0.09,0.120852,-0.0153018,-0.00418709,0.3, 0.05 ,0))
        self.collisionAvoidance.createcollisionlink("skin_28","box","internal",(0.25,0.09,0.09,0.119741,-0.0385305,-0.0190389,0.83,-0.04,-0))
        self.collisionAvoidance.createcollisionlink("skin_30","box","internal",(0.25,0.09,0.09,0.119756,-0.0512256,-0.0430332,1.52, 0. ,-0))
        self.collisionAvoidance.createcollisionlink("skin_47","box","internal",(0.25,0.09,0.09,0.119756,-0.0512256,-0.0430332,1.52, 0. ,-0))
        self.collisionAvoidance.createcollisionlink("skin_59","box","internal",(0.25,0.09,0.09,0.118712,-0.0446355,-0.0687232,2.02 ,0.,0))
        self.collisionAvoidance.createcollisionlink("skin_62","box","internal",(0.25,0.09,0.09,0.119505,-0.0267679,-0.0889594,2.55,-0.02,-0))
        self.collisionAvoidance.createcollisionlink("skin_82","box","internal",(0.25,0.09,0.09,0.119639,-0.00112243,-0.0964335,3.11,0.07,-0))
        '''
        # skin redefinition
        self.collisionAvoidance.createcollisionlink("skin_101","box","internal",(0.25,0.09,0.09,0,0,0,0.653, 0.590, 2.678))
        self.collisionAvoidance.createcollisionlink("skin_111","box","internal",(0.25,0.09,0.09,0,0,0,1.213, 0.590, 2.678))
        self.collisionAvoidance.createcollisionlink("skin_83","box","internal",(0.25,0.09,0.09,0,0,0,-0.467, 0.590, 2.678))
        self.collisionAvoidance.createcollisionlink("skin_87","box","internal",(0.25,0.09,0.09,0,0,0,0.093, 0.590, 2.678))
        self.collisionAvoidance.createcollisionlink("skin_14","box","internal",(0.25,0.09,0.09,0,0,0,2.385, 0.600, 2.658))
        self.collisionAvoidance.createcollisionlink("skin_28","box","internal",(0.25,0.09,0.09,0,0,0,2.435, 0.600, 2.658))
        self.collisionAvoidance.createcollisionlink("skin_30","box","internal",(0.25,0.09,0.09,0,0,0,2.910, 0.646, 2.562))
        self.collisionAvoidance.createcollisionlink("skin_47","box","internal",(0.25,0.09,0.09,0,0,0,-2.658, 0.626, 2.606))
        self.collisionAvoidance.createcollisionlink("skin_59","box","internal",(0.25,0.09,0.09,0,0,0,-2.158, 0.626, 2.606))
        self.collisionAvoidance.createcollisionlink("skin_62","box","internal",(0.25,0.09,0.09,0,0,0,-1.641, 0.636, 2.584))
        self.collisionAvoidance.createcollisionlink("skin_82","box","internal",(0.25,0.09,0.09,0,0,0,-1.027, 0.590, 2.678))       
        # hand
        self.collisionAvoidance.createcollisionlink("hand","box","external",(0.25,0.01,0.01,0.22,-1000,-1000.0,1000,0,0))
        self.collisionAvoidance.createcollisionpair("skin_101","hand")
        self.collisionAvoidance.createcollisionpair("skin_111","hand")
        self.collisionAvoidance.createcollisionpair("skin_83","hand")
        self.collisionAvoidance.createcollisionpair("skin_87","hand")
        self.collisionAvoidance.createcollisionpair("skin_14","hand")
        self.collisionAvoidance.createcollisionpair("skin_28","hand")
        self.collisionAvoidance.createcollisionpair("skin_30","hand")
        self.collisionAvoidance.createcollisionpair("skin_47","hand")
        self.collisionAvoidance.createcollisionpair("skin_59","hand")
        self.collisionAvoidance.createcollisionpair("skin_62","hand")
        self.collisionAvoidance.createcollisionpair("skin_82","hand")
        
        # 
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_101,self.collisionAvoidance.skin_101)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_111,self.collisionAvoidance.skin_111)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_83,self.collisionAvoidance.skin_83)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_87,self.collisionAvoidance.skin_87)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_14,self.collisionAvoidance.skin_14)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_28,self.collisionAvoidance.skin_28)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_30,self.collisionAvoidance.skin_30)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_47,self.collisionAvoidance.skin_47)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_59,self.collisionAvoidance.skin_59)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_62,self.collisionAvoidance.skin_62)
        plug(self.robot.dynamic.r_forerarm_skin_cell_joint_82,self.collisionAvoidance.skin_82)  
        
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_101,self.collisionAvoidance.Jskin_101)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_111,self.collisionAvoidance.Jskin_111)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_83,self.collisionAvoidance.Jskin_83)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_87,self.collisionAvoidance.Jskin_87)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_14,self.collisionAvoidance.Jskin_14)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_28,self.collisionAvoidance.Jskin_28)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_30,self.collisionAvoidance.Jskin_30)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_47,self.collisionAvoidance.Jskin_47)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_59,self.collisionAvoidance.Jskin_59)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_62,self.collisionAvoidance.Jskin_62)
        plug(self.robot.dynamic.Jr_forerarm_skin_cell_joint_82,self.collisionAvoidance.Jskin_82)  
                
        plug(self.robot.dynamic.r_elbow_joint,self.collisionAvoidance.hand)
        self.collisionAvoidance.Jhand.value = ((1,0,0,10000),(0,1,0,10000),(0,0,1,10000),(0,0,0,1))
        plug(self.ros.rosSubscribe.proximity,self.collisionAvoidance.proximitySensor) 
        
        self.task_skinsensor=TaskInequality('taskskinsensor')
        self.sensor_feature = FeatureGeneric('sensorfeature')
        plug(self.collisionAvoidance.collisionJacobian,self.sensor_feature.jacobianIN)
        plug(self.collisionAvoidance.collisionDistance,self.sensor_feature.errorIN)
        self.task_skinsensor.add(self.sensor_feature.name)
        self.task_skinsensor.referenceInf.value = (0.07,)*11
        self.task_skinsensor.referenceSup.value = (5.0,)*11
        self.task_skinsensor.dt.value=0.5
        #self.task_skinsensor.controlSelec.value = '110001000000000000000000000000000000000'
        self.task_skinsensor.controlGain.value = 2.5        
        
    
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
        
        
    # robot control procedures    
    def initializeRobot(self):
        self.connectDeviceWithSolver(False)
        if self.status == 'NOT_INITIALIZED':
            try:
                self.pushBasicTasks()
                #self.ps = PathSampler ('ps')
                #self.ps.createJointReference('l_wrist_roll_joint')
                #self.ps.loadRobotModel ('sot_robot', 'planar', 'Tom_sot')                
                self.status = 'INITIALIZED'
            except ValueError:
                self.status = 'NOT_INITIALIZED'

    def startRobot(self):
        if ((self.status == 'INITIALIZED') | (self.status == 'STOPPED')) :           
            #plug(self.robot.dynamic.position,self.ps.position)
            #self.ps.setTimeStep (0.01)
            self.status = 'STARTED'
            self.connectDeviceWithSolver(True) 
            self.posture_feature.posture.value = self.robot.device.state.value
           
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
