print("sot_robot")
print("Prologue loaded.")

'''
from dynamic_graph import plug
from dynamic_graph.sot.pr2.robot import Pr2
from dynamic_graph.sot.core import  SOT,FeaturePosition, Task

#from dynamic_graph.sot.youbot.youbot_device import YoubotDevice
from dynamic_graph.entity import PyEntityFactoryClass

Device = PyEntityFactoryClass('RobotDevice')
robot = Pr2(name = 'Pr2', device = Device('Pr2_device'))
dimension = robot.dynamic.getDimension()
plug(robot.device.state, robot.dynamic.position)

#solver#
solver = SOT ('solver')
solver.setSize (robot.dynamic.getDimension())
plug (solver.control, robot.device.control)

'''
#import sys
from dynamic_graph.sot.pr2.sot_interface import SOTInterface
sot = SOTInterface('Ur_device')
sot.connectDeviceWithSolver(False)

__all__ = ["sot"]

