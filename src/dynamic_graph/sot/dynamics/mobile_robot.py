#from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace

class AbstractMobileRobot(object):
    OperationalPoints = []
    AdditionalFrames = []   
    name = None
    initPosition = None
    tracer = None
    tracerSize = 2**20
    autoRecomputedSignals = []
    tracedSignals = {
        'dynamic': ["position", "velocity"],
        'device': ['control', 'state']
        }
    # initialize robot    
    def initializeRobot(self):
        if not self.dynamic:
            raise RunTimeError("robots models have to be initialized first")

        if not self.device:
            self.device = RobotSimu(self.name + '_device')

        self.device.set(self.initPosition)
        plug(self.device.state, self.dynamic.position)

        self.dynamic.velocity.value = self.dimension*(0.,)
        self.dynamic.acceleration.value = self.dimension*(0.,)

        self.initializeOpPoints(self.dynamic)

    # create operational points
    def initializeOpPoints(self, model):
        for op in self.OperationalPoints:
            model.createOpPoint(op, op)


    # Tracer methods
    def addTrace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append(
                '{0}.{1}'.format(entityName, signalName))
            addTrace(self, self.tracer, entityName, signalName)
    def startTracer(self):
        """
        Start the tracer if it does not already been stopped.
        """
        if self.tracer:
            self.tracer.start()
    def stopTracer(self):
        """
        Stop and destroy tracer.
        """
        if self.tracer:
            self.tracer.dump()
            self.tracer.stop()
            self.tracer.close()
            self.tracer.clear()
            for s in self.autoRecomputedSignals:
                self.device.after.rmSignal(s)
            self.tracer = None

    def initializeTracer(self,robotname):

        self.tracer = 0
        self.tracer = TracerRealTime('trace')
        self.tracer.setBufferSize(self.tracerSize)
        self.tracer.open(robotname,'dg_','.dat')
        # Recompute trace.triger at each iteration to enable tracing.
        self.device.after.addSignal('{0}.triger'.format(self.tracer.name))

    def traceDefaultSignals (self):
        # Geometry / operational points
        for s in self.OperationalPoints + self.tracedSignals['dynamic']:
            self.addTrace(self.dynamic.name, s)
        # Device
        for s in self.tracedSignals['device']:
            self.addTrace(self.device.name, s)
        if type(self.device) != RobotSimu:
            self.addTrace(self.device.name, 'robotState')

    # const and deconst
    def __init__(self, name, tracer = None):
        self.name = name
        # Initialize tracer if necessary.
        if tracer:
            self.tracer = tracer

    def __del__(self):
        if self.tracer:
            self.stopTracer()


class Ur(AbstractMobileRobot):
    initPosition = [0,0,0,0,0,0,0,0,0,0,0,0]
    def specifySpecialLinks(self):
        if len(self.SpecialLinks) == len(self.SpecialNames):
            for i in range(0,len(self.SpecialLinks)):
                self.dynamic.addJointMapping(self.SpecialLinks[i], self.SpecialNames[i])
        else:
            print 'No Special joints added : SpecialLinks.size != SpecialJoints.size'
    
    def __init__(self, name, device = None, tracer = None):
        AbstractMobileRobot.__init__ (self, name, tracer)
        # add operational points
        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('base_joint')
        self.OperationalPoints.append('shoulder_pan_joint')
        self.OperationalPoints.append('wrist_2_joint')
        self.OperationalPoints.append('wrist_3_joint')
        self.OperationalPoints.append('wrist_1_joint')
        self.OperationalPoints.append('shoulder_lift_joint')
        self.OperationalPoints.append('elbow_joint')
        # device and dynamic model assignment
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        # load model
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.initPosition = (0.,) * self.dimension
        # initialize ur robot
        self.initializeRobot()
__all__ = ["Ur"]




