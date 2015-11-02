import time
import rospy
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, FeaturePosture, Task, SOT, GainAdaptive
from dynamic_graph.sot.core.meta_tasks import generic6dReference
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import SolverKine
from rqt_kws.planning_interface import KineoPlanner_Interface
from dynamic_graph.ros import Ros
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts

#usage
#from dynamic_graph.sot.pr2.sot_interface import SOTInterface
#test = SOTInterface()

class SOTInterface:
    def __init__(self): 
        # define robot device
        self.robot = Pr2('Pr2', device=RobotSimu('Pr2'))
        self.dimension = robot.dynamic.getDimension()
        self.robot.device.resize (self.dimension)
        self.ros = Ros(self.robot)
        # define SOT solver
        self.solver = SolverKine('sot_solver')
        self.solver.setSize (robot.dynamic.getDimension())
        self.robot.device.resize (self.robot.dynamic.getDimension())
        # define basic tasks
        self.joint_names = rospy.get_param('sot_controller/jrl_map') 
        self.defineBasicTasks()
        self.pushBasicTasks()
        self.runner=inc()
        self.runner.once()
        [go,stop,next,n]=loopShortcuts(runner)
        
    def defineBasicTasks(self):
        # 1. DEFINE JOINT LIMIT TASK
        self.robot.dynamic.upperJl.recompute(0)
        self.robot.dynamic.lowerJl.recompute(0)
        ll = list(self.robot.dynamic.lowerJl.value)
        ul = list(self.robot.dynamic.upperJl.value)
        ll[13] = -2.1213
        ll[15] = -2
        ul[13] = -0.15
        ul[15] = -0.1
        self.jltaskname = self.defineJointLimitsTask(ll,ul)

        # 2. DEFINE BASEWAIST POSITIONING TASK
        position = [0,0,0]
        self.waisttaskname = self.defineWaistPositionTask(position)
        
        # 3. DEFINE ROBOT POSTURE TASK
        posture = [0.0]*self.dimension
        self.posturetaskname = self.defineRobotPostureTask(posture)
    
    def pushBasicTasks(self):
        solver.push(self.jltaskname)
        solver.push(self.waisttaskname)
        solver.push(self.posturetaskname)
        connectDeviceWithSolver(True)

     @loopInThread
     def inc():
        self.robot.device.increment(dt)




    def pushTask(self,task):
        solver.push(task.name)

    def clearSolver(self)
        connectDeviceWithSolver(False)
        solver.clear()


    def connectDeviceWithSolver(self,switch=False):
        if switch == True:
            plug (self.solver.control,self.robot.device.control)
        else:
            robot.device.control.unplug()


    def defineJointLimitsTask(self,inf=None,sup=None):
        # joint limits
        self.taskjl = TaskJointLimits('Joint Limits Task')
        plug(self.robot.dynamic.position,self.taskjl.position)
        taskjl.controlGain.value = 5
        taskjl.referenceInf.value = ll
        taskjl.referenceSup.value = ul
        taskjl.dt.value = 1
        return task_jl.name
        

    def defineWaistPositionTask(self,position):
        task_waist_metakine=MetaTaskKine6d('Waist Task',robot.dynamic,'base_joint','base_joint')
        self.goal_waist = ((1.,0,0,postion[0]),(0,1.,0,postion[1]),(0,0,1.,postion[2]),(0,0,0,1.),)
        task_waist_metakine.feature.frame('desired')
        #task_waist_metakine.feature.selec.value = '111101'#RzRyRxTzTyTx
        task_waist_metakine.gain.setConstant(10)
        task_waist_metakine.featureDes.position.value = self.goal_waist
        return task_waist_metakine.task.name

    def setWaistPosition(self,position):
        self.goal_waist = ((1.,0,0,postion[0]),(0,1.,0,postion[1]),(0,0,1.,postion[2]),(0,0,0,1.),)

    def defineRobotPostureTask(self,posture=None)
        self.posture_feature = FeaturePosture('featurePosition')
        plug(self.robot.device.state,self.posture_feature.state)

        if ((len(posture) != self.dimension) :
            posture = robot.device.state
        self.posture_feature.posture.value = posture 

        postureTaskDofs = [True]*(39)
        for dof,isEnabled in enumerate(postureTaskDofs):
            if dof > 5:
              self.posture_feature.selectDof(dof,isEnabled)
        task_posture=Task('Posture Task')
        task_posture.add(posture_feature.name)
        # featurePosition.selec.value = toFlags((6,24))
        gainPosition = GainAdaptive('gainPosition')
        gainPosition.set(0.1,0.1,125e3)
        gainPosition.gain.value = 5
        plug(task_posture.error,gainPosition.error)
        plug(gainPosition.gain,task_posture.controlGain)
        return task_posture.name
    
    def setRobotPosture(self,posture):
        self.posture_feature.posture.value = posture




