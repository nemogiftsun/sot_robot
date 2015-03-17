# -*- coding: utf-8 -*-
# Copyright 2013, Benjamin Coudrin, LIRMM, CNRS
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#from dynamic_graph.sot.dynamics.abstract_robot import AbstractRobot
from dynamic_graph.sot.dynamics.mobile_robot import AbstractMobileRobot
from dynamic_graph.ros.robot_model import RosRobotModel
from dynamic_graph.sot.core import  SOT,FeaturePosition, Task
class youbot(AbstractMobileRobot):
    """
    This class instanciates a Ur5 robot.
    """

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    #initPosition = [0,0,0,0,0,0,0.011,0.011,-0.1, 0.023, 0.12,0,0]
    
    def specifySpecialLinks(self):
        if len(self.SpecialLinks) == len(self.SpecialNames):
            for i in range(0,len(self.SpecialLinks)):
                self.dynamic.addJointMapping(self.SpecialLinks[i], self.SpecialNames[i])
        else:
            print 'No Special joints added : SpecialLinks.size != SpecialJoints.size'

    def __init__(self, name, device = None, tracer = None):
        AbstractMobileRobot.__init__ (self, name, tracer)
        # add operational points
        ip = (0,0,0,0,0,0.0022,0,-0.00012,0.00407,0.00345,0.001555,0.0007879)
        self.OperationalPoints.append('base_joint')
        self.OperationalPoints.append('arm_joint_1')
        self.OperationalPoints.append('arm_joint_2')
        self.OperationalPoints.append('arm_joint_3')      
        self.OperationalPoints.append('arm_joint_4')
        self.OperationalPoints.append('arm_joint_5')
        self.OperationalPoints.append('gripper_finger_joint_l')
        self.OperationalPoints.append('gripper_finger_joint_r')
        # device and dynamic model assignment
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        # load model
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        self.initPosition = ip
        #self.initPosition = (0.,) * self.dimension
        # initialize ur robot
        self.initializeRobot()        
__all__ = ["youbot"]

