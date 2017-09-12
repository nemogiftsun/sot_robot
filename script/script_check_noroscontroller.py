from dynamic_graph.sot.ur.sot_interface_v3 import SOTInterface
from dynamic_graph import plug, writeGraph
test = SOTInterface()
test.startRobot()

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    test.robot.device.increment(0.01)

runner=inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)

import time

time.sleep(1)

go
