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
class Tom(AbstractMobileRobot):
    """
    This class instanciates a Tom robot.
    """

    tracedSignals = {
        'dynamic': ["com", "position", "velocity", "acceleration"],
        'device': ['control', 'state']
        }
        
    #initPosition = [0,0,0,0,0,0,0.011,0.011,-0.1, 0.023, 0.12,0,0]
    
    def __init__(self, name, device = None, tracer = None):
        AbstractMobileRobot.__init__ (self, name, tracer)
        #self.urdfDir = rospack.get_path('tom_description') + '/robots/'
        #self.urdfName = 'tom_lacquey.urdf.xacro'
        # add operational points
        self.OperationalPoints.append('base_joint')
        #self.OperationalPoints.append('l_s') 
        '''       
        self.OperationalPoints.append('l_shoulder_pan_joint')
        self.OperationalPoints.append('l_shoulder_lift_joint')
        self.OperationalPoints.append('l_elbow_joint')
        self.OperationalPoints.append('l_wrist_1_joint')
        self.OperationalPoints.append('l_wrist_2_joint')
        self.OperationalPoints.append('l_wrist_3_joint')
        '''
        self.OperationalPoints.append('r_shoulder_pan_joint')
        self.OperationalPoints.append('r_shoulder_lift_joint')
        self.OperationalPoints.append('r_elbow_joint')
        '''
        self.OperationalPoints.append('r_forerarm_skin_root_cell_joint')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_101')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_111')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_83')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_87')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_14')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_28')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_30')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_47')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_59')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_62')
        self.OperationalPoints.append('r_forerarm_skin_cell_joint_82')
        self.OperationalPoints.append('r_wrist_1_joint')
        self.OperationalPoints.append('r_wrist_2_joint')
        self.OperationalPoints.append('r_wrist_3_joint')
        self.OperationalPoints.append('r_front_wheel_joint')  
        '''
       # device and dynamic model assignment
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        #self.pinocchioModel = se3.buildModelFromUrdf(self.urdfDir + self.urdfName,
        #                                             se3.JointModelFreeFlyer())
        #self.pinocchioData = self.pinocchioModel.createData()
        #self.dynamic.setModel(self.pinocchioModel)
        #self.dynamic.setData(self.pinocchioData)
        # load model
        self.dynamic.loadFromParameterServer()
        self.dimension = self.dynamic.getDimension()
        #self.initPosition = ip
        self.initPosition = (0.,) * self.dimension
        # initialize ur robot
        self.initializeRobot()        
__all__ = ["Tom"]

