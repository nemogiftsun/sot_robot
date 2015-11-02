import os
import rospy
import rospkg
import tf

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, QThread, QTimer, QMutex

from sensor_msgs.msg import JointState


import xml.etree.ElementTree as ET
file = '/home/nemogiftsun/laasinstall/devel/ros/src/sot_robot/src/rqt_rpc/rpc_config.xml'



from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.pr2.sot_interface import SOTInterface
from rqt_kws.planning_interface import KineoPlanner_Interface
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts


from dynamic_graph.sot.hpp import PathSampler


import math
## History
# loopShortcuts
# Threading

########### CHUNK##################
#sot = SOTInterface()addway
#########
#base=[0,0,0,0,0,0]
#torso=[0,0]
#head = [0,0,0]
#right_arm = [0.0, 0.0, 0.0, -0.1499999999999993, 0.0, -0.09999999999999967,0,0,0,0,0,0,0,0]
#left_arm = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#jp = base+[torso[0]]+ head+right_arm+left_arm+ [torso[1]]
# Controller Thread
#@loopInThread
#def inc():
#    sot.robot.device.increment(0.1)


#runner=inc()
#runner.once()
#[go,stop,next,n]=loopShortcuts(runner)
#g11

#sot.setRobotPosture(jp)
#############################################

order = ('', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'r_upper_arm_roll_joint', '', '', '', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', '')


#[0,0,0,state[order.index('l_shoulder_pan_joint')], state[order.index('l_shoulder_lift_joint')], state[order.index('l_upper_arm_roll_joint')],state[order.index('l_elbow_flex_joint')], state[order.index('l_forearm_roll_joint')], state[order.index('l_wrist_flex_joint')], state[order.index('l_wrist_roll_joint')], state[order.index('l_gripper_l_finger_joint')], state[order.index('l_gripper_l_finger_tip_joint')],state[order.index('l_gripper_motor_slider_joint')],state[order.index( 'l_gripper_motor_screw_joint')], state[order.index('l_gripper_r_finger_joint')], state[order.index('l_gripper_r_finger_tip_joint')], state[order.index('l_gripper_joint')],0,state[order.index('r_shoulder_pan_joint')], state[order.index('r_shoulder_lift_joint')], state[order.index('r_upper_arm_roll_joint')], state[order.index('r_elbow_flex_joint')], state[order.index('r_forearm_roll_joint')], state[order.index('r_wrist_flex_joint')], state[order.index('r_wrist_roll_joint')], state[order.index('r_gripper_l_finger_joint')], state[order.index('r_gripper_l_finger_tip_joint')],state[order.index('r_gripper_motor_slider_joint')],state[order.index( 'r_gripper_motor_screw_joint')], state[order.index('r_gripper_r_finger_joint')], state[order.index('r_gripper_r_finger_tip_joint')], state[order.index('r_gripper_joint')],0]





class RPCPlugin(Plugin):
    mutex = QMutex()
    def __init__(self, context):
        super(RPCPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SOT Control')   
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('sot_robot'), 'resource', 'rpc_gui.ui')
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('RPCPluginUi')
        if context.serial_number() > 1:
            return None

        #self.planner = KineoPlanner_Interface()
        self.init = [] 
        self.goal = []

        self.status_plan = 'NOT_REQUESTED'

        self.status_robot = 'NOT INITIALIZED'

        self.change = 0.000

        # Connect button
        self._widget.Start.clicked[bool].connect(self._start_robot)

        self._widget.Stop.clicked[bool].connect(self._stop_robot)

        self._widget.InitializeRobot.clicked[bool].connect(self._initialize_robot)  

        self._widget.pushButton.clicked[bool].connect(self._change)

        self._widget.copyGoal.clicked[bool].connect(self._copyGoal)

        self._widget.copyInit.clicked[bool].connect(self._copyInit)

        self._widget.goInit.clicked[bool].connect(self._goInit)

        self._widget.goGoal.clicked[bool].connect(self._goGoal)

        self._widget.pleasePlan.clicked[bool].connect(self._pleasePlan)

        self._widget.executePlan.clicked[bool].connect(self._executePlan)

        # Scenario execution
        self._widget.goPreGrasp.clicked[bool].connect(self._goPreGrasp)
        self._widget.goInitial.clicked[bool].connect(self._goInitial)
        #self._widget.goGrasp.clicked[bool].connect(self._goGrasp)
        #self._widget.GoPostGrasp.clicked[bool].connect(self._goPostGrasp)

        # reset interpolation 
        self._widget.resetInterpolator.clicked[bool].connect(self._resetInterpolator)


        # display update``
        self.timer_update = QTimer(self)
        self.timer_update.timeout.connect(self._update_display)

        # control cycle
        self.timer_control_cycle = QTimer(self)
        self.timer_control_cycle.timeout.connect(self.increment)
        
        context.add_widget(self._widget)
        self.val = 0
        self.jp = []
        self.switch_plugged = False
       

        self.CF_tree = ET.parse(file)
        self.CF_root = self.CF_tree.getroot()
        if (self.CF_root.tag == "Trajectories"):
            self.CF_trajectories = self.CF_root.findall('trajectory')
            self.CF_count = len(self.CF_trajectories)

        for trajectory in self.CF_trajectories:
            self._widget.Trajectories.addItem(str(trajectory.attrib['name']))
    


        # coded information of the joint dofs.
        self.code = [7,9,13,22,24,28,32]
        self.posesignalstring = 'self.sot.robot.device.state'

        self.sh = RosShell()


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


    def _goInitial(self):
        for trajectory in self.CF_trajectories:
            if trajectory.attrib['name'] == self._widget.Trajectories.currentText():
                self.wps = trajectory.findall('waypoint')
                break        
        for wp in self.wps:
            if wp.attrib['name'] == 'initial_configuration':
                sot_config = [float(x) for x in wp.attrib['jc'].split(',')]
                self.init = sot_config 
                break
        self._goInit()
        


    def _goPreGrasp(self):
        self.switch_plugged = True
        self.ps.resetPath()
        self.wps = ()
        for trajectory in self.CF_trajectories:
            if trajectory.attrib['name'] == self._widget.Trajectories.currentText():
                self.wps = trajectory.findall('waypoint')
                break

        for wp in self.wps:
            sot_config = [float(x) for x in wp.attrib['jc'].split(',')]
            hpp_config = self._converttohpp(sot_config)
            self.ps.addWaypoint(tuple(hpp_config))
            if wp.attrib['name'] == 'pregrasp_configuration':
                break
        plug(self.sot.robot.device.state, self.ps.position)
        self.ps.setTimeStep (0.01)
        self.ps.configuration.recompute(self.ps.configuration.time)
        self.ps.start()
        plug(self.ps.configuration,self.sot.posture_feature.posture) 
        '''
        self.timer_control_cycle.stop()
        plug(self.ps.configuration,self.sot.posture_feature.posture) 
        self.posesignalstring = 'self.ps.'+ self.referenceSignal
        #self.sot.definePoseTask(self.referenceSignal)
        plug(eval(self.posesignalstring),self.sot.task_pose_metakine.featureDes.position)
        self.sot.changeDefaultStackToRPP()
        self.sot.connectDeviceWithSolver(True)
        self.timer_control_cycle.start()
        '''




    def _copyGoal(self):
        self._copyState('goal')

    def _copyInit(self):
        self._copyState('init')

    def _copyState(self,point): 
        #print type(state[6:len(state)])
        state = self.sot.robot.dynamic.position.value
        print state
        '''
        pose = state[0:6]
        th = state[6:9]
        l = state[9:23]
        laser = state[23]
        r = state[24:38]
        if self.planningGroup == 'left_arm':
            state = l
        elif self.planningGroup == 'right_arm':
            state = r
        elif self.planningGroup == 'left_full_body':
            state = pose+l
        
        jointsList = self.planner.mvi.get_group_joint_names(self.planningGroup)  
        jointState = ()
        if ('world_joint' in jointsList): 
            jointState = self.sot.robot.dynamic.position.value[0:6]

        jointState = jointState + self.sot.robot.dynamic.position.value[6:39]
             
        j = 0
        state = [0,]*len(jointsList)
        for joint in jointsList:
            i = self.sot.joint_names.index(joint)
            state[j] = jointState[i]
            j = j + 1
        '''
        if point == 'goal': 
            self.goal = state
            self._widget.jointstate_goal.setText(str(self.goal))
        elif point == 'init':
            self.init = state
            self._widget.jointstate_init.setText(str(self.init))
        #print self.init

    def _goGoal(self):
        #self._widget.jointstate.setText(str(self.goal['dof'].split(',')))
        #state = [float(i) for i in self.goal['dof'].split(',')[6:45]]
        #state = state[0:6] + state[6:9] + state[24:38]+[state[9],]+state[10:24]+[state[38],]
        #self.sot.posture_feature.posture.unplug()
        self.switch_plugged = False
        self._goState(self.goal)


    def _goInit(self):
        #self._widget.jointstate.setText(str(self.init['dof'].split(',')))
        #state = [float(i) for i in self.init['dof'].split(',')[6:45]]
        #print state
        #self._widget.jointstate.setText(str(state))
        #state = state[0:6] + state[6:9] + state[24:38]+[state[9],]+state[10:24]+[state[38],]
        #print len(state)
        self.switch_plugged = False
        self._goState(self.init)


    def _goState(self,state):
        #print  self.init
        self._widget.base_x_joint.setValue(state[0])
        self._widget.base_y_joint.setValue(state[1])
        self._widget.base_yaw_joint.setValue(state[5])
        self._widget.torso_lift_joint.setValue(state[6])  
        self._widget.head_pan_joint.setValue(state[7])  
        self._widget.head_tilt_joint.setValue(state[8]) 
        self._widget.l_shoulder_pan_joint.setValue(state[9])
        self._widget. l_shoulder_lift_joint.setValue(state[10])
        self._widget.l_upper_arm_roll_joint.setValue(state[11])
        self._widget.l_elbow_flex_joint.setValue(state[12]) 
        self._widget.l_forearm_roll_joint.setValue(state[13]) 
        self._widget.l_wrist_flex_joint.setValue(state[14])  
        self._widget.l_wrist_roll_joint.setValue(state[15])  
        self._widget.l_gripper_l_finger_joint.setValue(state[16])  
        self._widget.l_gripper_l_finger_tip_joint.setValue(state[17]) 
        self._widget.l_gripper_motor_slider_joint.setValue(state[18])  
        self._widget.l_gripper_motor_screw_joint.setValue(state[19])  
        self._widget.l_gripper_r_finger_joint.setValue(state[20])  
        self._widget.l_gripper_r_finger_tip_joint.setValue(state[21]) 
        self._widget.l_gripper_joint.setValue(state[22]) 
        self._widget.r_shoulder_pan_joint.setValue(state[24])  
        self._widget.r_shoulder_lift_joint.setValue(state[25]) 
        self._widget.r_upper_arm_roll_joint.setValue(state[26])  
        self._widget.r_elbow_flex_joint.setValue(state[27])  
        self._widget.r_forearm_roll_joint.setValue(state[28])  
        self._widget.r_wrist_flex_joint.setValue(state[29])  
        self._widget.r_wrist_roll_joint.setValue(state[30])  
        self._widget.r_gripper_l_finger_joint.setValue(state[31])  
        self._widget.r_gripper_l_finger_tip_joint.setValue(state[32])  
        self._widget.r_gripper_motor_slider_joint.setValue(state[33])  
        self._widget.r_gripper_motor_screw_joint.setValue(state[34])  
        self._widget.r_gripper_r_finger_joint.setValue(state[35])  
        self._widget.r_gripper_r_finger_tip_joint.setValue(state[36])  
        self._widget.r_gripper_joint.setValue(state[37])
        self._widget.pushButton.click()  

 
    def _pleasePlan(self): 
        self.jointsList = self.planner.mvi.get_group_joint_names(self.planningGroup)       
        j = 0
        i = 0
        init = [0,]*len(self.jointsList) 
        goal = [0,]*len(self.jointsList) 

        for joint in self.jointsList:
            
            if joint == 'world_joint': 
                init = [0,]*(6+len(self.jointsList)) 
                goal = [0,]*(6+len(self.jointsList)) 
                for i in range(6):              
                    init[j] = self.init[i]
                    goal[j] = self.goal[i]  
                    j = j + 1
            else:
                i = self.sot.joint_names.index(joint)
                init[j] = self.init[i+6]
                goal[j] = self.goal[i+6]
                j = j + 1


        self.planner.callPathPlanService(self.planningGroup,init,goal)

        self.trajectory_size = self.planner.sizeTrajectory()
        if (self.trajectory_size > 0):
            self.status_plan = "PLAN SUCCESSFUL"
        else:
            self.status_plan = "PLANNING NOT SUCCESSFUL"



    def _executefirst(self):
        self.ps.resetPath()
        trajectory_point_init = (0.0, 0.0, 1.0, 0.0, 0.10999999999999986, 0.0, 0.0, 1.5000000000000022, 0.0, 1.550000000000006, -0.5499971498500139, 0.9581582649409572, 0.28623895494033386, -0.3799999676557602, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.9999999999999978, 0.9999999999999989, 0.0, -1.9299999999999977, 1.0, 0.0, -0.10999999999999986, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        trajectory_point_grasp = (0.0, 0.0, 1.0, 0.0, 0.10999999999999986, 0.0, 0.0, 1.5000000000000022, 0.0, 1.550000000000006, -0.5499971498500139, 0.9581582649409572, 0.28623895494033386, -0.3799999676557602, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.9999999999999978, 0.9999999999999989, 0.0, -1.9299999999999977, 1.0, 0.0, -0.10999999999999986, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        self.ps.setTimeStep (0.1)
        self.ps.configuration.recompute(self.ps.configuration.time)
        self.ps.start()
        self.timer_control_cycle.stop()
        plug(self.ps.configuration,self.sot.posture_feature.posture) 
        self.posesignalstring = 'self.ps.'+ self.referenceSignal
        self.sot.definePoseTask(self.referenceSignal)
        plug(eval(self.posesignalstring),self.sot.task_pose_metakine.featureDes.position)
        self.sot.changeDefaultStackToRPP()
        self.sot.connectDeviceWithSolver(True)
        self.timer_control_cycle.start()


    def _executePlan(self):
        #print self.planner.getTrajectoryPointJointNames()
        self.switch_plugged = True
        self.ps.resetPath()
        trajectory_point = ()
        multi_dof_trajectory = self.planner.getMultiDOFTrajectory()
        pose = ()
        for n in range(self.trajectory_size):
            trajectory_joint_state = self.planner.getTrajectoryPoint(n)
            #print trajectory_joint_state
            current_robot_joint_state = self.sot.robot.dynamic.position.value[6:39]
            trajectory_point = self._converttohpp(self.sot.robot.dynamic.position.value)
            j = 0
            if 'world_joint' in self.jointsList:
                index_wj = multi_dof_trajectory.joint_names.index('world_joint')
                transform = multi_dof_trajectory.points[index_wj].transforms[n]
                trajectory_point[j] = transform.translation.x
                trajectory_point[j+1] = transform.translation.y 
                rpy = tf.transformations.euler_from_quaternion(transform.rotation)
                cos_baseyaw = math.cos(rpy(2))
                sin_baseyaw = math.sin(rpy(2))
                trajectory_point[j+2] = cos_baseyaw
                trajectory_point[j+3] = sin_baseyaw         
            j = 4
            for joint in self.sot.joint_names:
                i = self.sot.joint_names.index(joint)
                if (i in self.code):
                    if (joint in self.jointsList):
                        index  = self.jointsList.index(joint)
                        cos_t = math.cos(trajectory_joint_state[index])
                        sin_t = math.sin(trajectory_joint_state[index])
                        trajectory_point[j] = cos_t
                        trajectory_point[j+1] = sin_t
                    j = j + 2
                else:
                    if (joint in self.jointsList):
                        index  = self.jointsList.index(joint)
                        trajectory_point[j] = trajectory_joint_state[index]
                    j = j + 1
            print tuple(trajectory_point)
            self.ps.addWaypoint (tuple(trajectory_point))

        self.ps.createJointReference(self.referenceSignal)
        plug(self.sot.robot.device.state, self.ps.position)
        self.ps.setTimeStep (0.1)
        self.ps.configuration.recompute(self.ps.configuration.time)
        self.ps.start()
        self.timer_control_cycle.stop()
        plug(self.ps.configuration,self.sot.posture_feature.posture) 
        self.posesignalstring = 'self.ps.'+ self.referenceSignal
        self.sot.definePoseTask(self.referenceSignal)
        plug(eval(self.posesignalstring),self.sot.task_pose_metakine.featureDes.position)
        self.sot.changeDefaultStackToRPP()
        self.sot.connectDeviceWithSolver(True)
        self.timer_control_cycle.start()

        #self.sot.connectDeviceWithSolver(True)


        #self.sot.setRobotPosture(self.ps.configuration.value)
        #self.sot.posture_feature.posture.value = self.ps.configuration.value
        #self.sot.setRobotPosture(list(self.ps.configuration.value))
        #print self.ps.configuration.value
        #self._goState(self.ps.configuration.value)
    
      
        


    def _resetInterpolator(self):
        self.sot.posture_feature.posture.value  = self.sot.robot.dynamic.position.value 
        self.ps.resetPath()
        


    def increment(self):
        #self.mutex.lock()
        self.sot.robot.device.increment(0.1)
        #print self.ps.configuration.value
        #self.mutex.unlock()
    
    def _update_display(self):
        # update joint state
        #self._widget.jointstate.setText(str(self.sot.robot.dynamic.position.value))
        state = [str(i) for i in self.sot.robot.dynamic.position.value]
        self._widget.base_x_joint_state.setText(state[0])
        self._widget.base_y_joint_state.setText(state[1])
        self._widget.base_yaw_joint_state.setText(state[5])
        self._widget.torso_lift_joint_state.setText(state[6])  
        self._widget.head_pan_joint_state.setText(state[7])  
        self._widget.head_tilt_joint_state.setText(state[8]) 
        self._widget.l_shoulder_pan_joint_state.setText(state[9])
        self._widget. l_shoulder_lift_joint_state.setText(state[10])
        self._widget.l_upper_arm_roll_joint_state.setText(state[11])
        self._widget.l_elbow_flex_joint_state.setText(state[12]) 
        self._widget.l_forearm_roll_joint_state.setText(state[13]) 
        self._widget.l_wrist_flex_joint_state.setText(state[14])  
        self._widget.l_wrist_roll_joint_state.setText(state[15])  
        self._widget.l_gripper_l_finger_joint_state.setText(state[16])  
        self._widget.l_gripper_l_finger_tip_joint_state.setText(state[17]) 
        self._widget.l_gripper_motor_slider_joint_state.setText(state[18])  
        self._widget.l_gripper_motor_screw_joint_state.setText(state[19])  
        self._widget.l_gripper_r_finger_joint_state.setText(state[20])  
        self._widget.l_gripper_r_finger_tip_joint_state.setText(state[21]) 
        self._widget.l_gripper_joint_state.setText(state[22]) 
        self._widget.r_shoulder_pan_joint_state.setText(state[24])  
        self._widget.r_shoulder_lift_joint_state.setText(state[25]) 
        self._widget.r_upper_arm_roll_joint_state.setText(state[26])  
        self._widget.r_elbow_flex_joint_state.setText(state[27])  
        self._widget.r_forearm_roll_joint_state.setText(state[28])  
        self._widget.r_wrist_flex_joint_state.setText(state[29])  
        self._widget.r_wrist_roll_joint_state.setText(state[30])  
        self._widget.r_gripper_l_finger_joint_state.setText(state[31])  
        self._widget.r_gripper_l_finger_tip_joint_state.setText(state[32])  
        self._widget.r_gripper_motor_slider_joint_state.setText(state[33])  
        self._widget.r_gripper_motor_screw_joint_state.setText(state[34])  
        self._widget.r_gripper_r_finger_joint_state.setText(state[35])  
        self._widget.r_gripper_r_finger_tip_joint_state.setText(state[36])  
        self._widget.r_gripper_joint_state.setText(state[37])



        # update the stack of tasks
        self._update_stack()

        self._widget.configuration.clear()
        #self._widget.configuration.setText(str(self.sot.collisionAvoidance.collisionDistance.value) )
        #self._widget.configuration.setText(str(self.sot.robot.dynamic.position.value) )
        self.planningGroup = str(self._widget.planningGroup.currentText())
        self.referenceSignal = str(self._widget.referenceSignal.currentText())
        self.referencePoseSignal = str(self._widget.referenceSignal.currentText())
        # planning
        self._widget.planlogger.setText(str(self.status_plan))

    def _update_stack(self): 
        self._widget.Stack.clear()
        if((self.status_robot == 'STARTED') or (self.status_robot == 'INITIALIZED')):           
            lstack = list(self.sot.solver.dispStack())
            matching = [s for s in range(len(lstack)) if "|" in lstack[s]]  
            num_tasks = len(matching) 
            for i in range(num_tasks):
                l = matching[i] + 1
                if (i+1 == num_tasks):
                    u = len(lstack) - 1
                else:
                    u = matching[i+1]-1
                self._widget.Stack.append('L'+str(i)+'<'+''.join(lstack[l:u])+'>')
        else:
            self._widget.Stack.append('<'+'STACK EMPTY'+'>') 


    def _change(self, checked):
        self._widget.display.clear()
        self.val = self.val + 1
        self._widget.display.append(str(self.val))
        self.larm = [self._widget.l_shoulder_pan_joint.value(),self._widget. l_shoulder_lift_joint.value(),self._widget.l_upper_arm_roll_joint.value(),self._widget.l_elbow_flex_joint.value(),self._widget.l_forearm_roll_joint.value(),self._widget.l_wrist_flex_joint.value(),self._widget.l_wrist_roll_joint.value(),self._widget.l_gripper_l_finger_joint.value(),self._widget.l_gripper_l_finger_tip_joint.value(),self._widget.l_gripper_motor_slider_joint.value(),self._widget.l_gripper_motor_screw_joint.value(),self._widget.l_gripper_r_finger_joint.value(),self._widget.l_gripper_r_finger_tip_joint.value(),self._widget.l_gripper_joint.value()]
        self.rarm = [self._widget.r_shoulder_pan_joint.value(),self._widget.r_shoulder_lift_joint.value(),self._widget.r_upper_arm_roll_joint.value(),self._widget.r_elbow_flex_joint.value(),self._widget.r_forearm_roll_joint.value(),self._widget.r_wrist_flex_joint.value(),self._widget.r_wrist_roll_joint.value(),self._widget.r_gripper_l_finger_joint.value(),self._widget.r_gripper_l_finger_tip_joint.value(),self._widget.r_gripper_motor_slider_joint.value(),self._widget.r_gripper_motor_screw_joint.value(),self._widget.r_gripper_r_finger_joint.value(),self._widget.r_gripper_r_finger_tip_joint.value(),self._widget.r_gripper_joint.value()]
        
        self.jp = [self._widget.base_x_joint.value(),self._widget.base_y_joint.value(),0,0,0,self._widget.base_yaw_joint.value()]+[self._widget.torso_lift_joint.value(),self._widget.head_pan_joint.value(),self._widget.head_tilt_joint.value()]+self.larm+[0]+self.rarm+[0]
        #self.mutex.lock()
        #self.mutex.unlock()
        self.sot.task_waist_metakine.featureDes.position.value = ((1.,0,0,self._widget.base_x_joint.value()),(0,1.,0,self._widget.base_y_joint.value()),(0,0,1.,0),(0,0,0,1.),)
        if self.switch_plugged == False:
              self.sot.setRobotPosture(self.jp)

      
    def _start_robot(self, checked): 
        self.sot.connectDeviceWithSolver(True)    
        self._widget.Logger.append(self.status_robot)
        self.status_robot = 'STARTED'
        self._widget.Logger.append(self.status_robot)
        self.jp = self.sot.robot.dynamic.position.value
        plug(self.sot.robot.dynamic.position,self.ps.position)
        self.ps.setTimeStep (0.001)
        # start timers
        self.timer_update.start(500)
        self.timer_control_cycle.start(100)
        self._widget.pushButton.click() 


    def _stop_robot(self, checked):   
        self.sot.connectDeviceWithSolver(False) 
        self.status_robot = 'STOPPED'
        self._widget.Logger.append(self.status_robot)
        # stop the controller
        self.timer_control_cycle.stop()


    def _initialize_robot(self, checked):  
        self._widget.Logger.append("Initializing")
        self.sot = SOTInterface()
        self.ps = PathSampler ('ps')
        self.referenceSignal = str(self._widget.referenceSignal.currentText())
        self.ps.createJointReference(self.referenceSignal)
        self.ps.loadRobotModel ('sot_robot', 'planar', 'pr2_sot')
        self.status_robot = 'INITIALIZED'
        self._widget.Logger.append(self.status_robot) 
        self.planner = KineoPlanner_Interface()       

        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


#############################################################################################################################################

#ROS SHELL

##############################################################################################################################################

import roslib; roslib.load_manifest('dynamic_graph_bridge')
import rospy

import dynamic_graph_bridge_msgs.srv

import sys
import code
import readline

class RosShell():
    def __init__(self):
        self.cache = ""
        rospy.loginfo('waiting for service...')
        rospy.wait_for_service('run_command')
        self.client = rospy.ServiceProxy(
            'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
    def runcode(self, code, retry = True):
        try:
            if not self.client:
                print("Connection to remote server lost. Reconnecting...")
                self.client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                if retry:
                    print("Retrying previous command...")
                    return self.runcode(code, False)
            response = self.client(code)
            print 'running'
            if response.standardoutput != "":
                print response.standardoutput[:-1]
            if response.standarderror != "":
                print response.standarderror[:-1]
            elif response.result != "None":
                print response.result
        except rospy.ServiceException, e:
            print("Connection to remote server lost. Reconnecting...")
            self.client = rospy.ServiceProxy(
                'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
            if retry:
                print("Retrying previous command...")
                self.runcode(code, False)















