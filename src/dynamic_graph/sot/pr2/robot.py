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
class Pr2(AbstractMobileRobot):
    """
    This class instanciates a Ur5 robot.
    """

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    #initPosition = [0,0,0,0,0,0,0.011,0.011,-0.1, 0.023, 0.12,0,0]
    
    def __init__(self, name, device = None, tracer = None):
        AbstractMobileRobot.__init__ (self, name, tracer)
        # add operational points
        ip = (0.,0.,0.,0.,0.,0.,0.011,0.0,0.0,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.)
        self.OperationalPoints.append('base_joint')
        self.OperationalPoints.append('r_shoulder_pan_joint')
        self.OperationalPoints.append('r_shoulder_lift_joint')
        self.OperationalPoints.append('l_shoulder_pan_joint')
        self.OperationalPoints.append('l_shoulder_lift_joint')
        self.OperationalPoints.append('r_wrist_roll_joint')
        self.OperationalPoints.append('l_wrist_roll_joint')
        self.OperationalPoints.append('r_wrist_flex_joint')
        self.OperationalPoints.append('l_wrist_flex_joint')
        self.OperationalPoints.append('torso_lift_joint')
        self.OperationalPoints.append('l_upper_arm_roll_joint')
        self.OperationalPoints.append('r_upper_arm_roll_joint')
        self.OperationalPoints.append('l_forearm_roll_joint')
        self.OperationalPoints.append('r_forearm_roll_joint')
        self.OperationalPoints.append('l_forearm_flex_joint')
        self.OperationalPoints.append('r_forearm_flex_joint')


#        self.OperationalPoints.append('arm_joint_2')
#        self.OperationalPoints.append('arm_joint_3')      
#        self.OperationalPoints.append('arm_joint_4')
#        self.OperationalPoints.append('arm_joint_5')

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
__all__ = ["Pr2"]

