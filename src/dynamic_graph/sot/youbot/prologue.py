print("sot_youbot")
print("Prologue loaded.")

from dynamic_graph import plug
from dynamic_graph.sot.youbot.robot import youbot
#from dynamic_graph.sot.youbot.youbot_device import YoubotDevice
from dynamic_graph.entity import PyEntityFactoryClass

Device = PyEntityFactoryClass('YoubotDevice')

robot = youbot(name = 'robot', device = Device('SoTYoubot'))
plug(robot.device.state, robot.dynamic.position)

__all__ = ["robot"]

