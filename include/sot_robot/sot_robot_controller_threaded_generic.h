
/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, LAAS-CNRS.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ROBOT_CONTROLLER_PLUGIN_H
#define ROUBOT_CONTROLLER_PLUGIN_H

#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointTrajectoryControllerState.h>


#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sot_robot/sot_robot_device.h>

// URDF
#include <urdf/model.h>

namespace sot_robot {

typedef boost::shared_ptr<hardware_interface::JointHandle> RobotJointPtr;
typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;

struct Commands
{
double position_; // Last commanded position
double velocity_; // Last commanded velocity
bool has_velocity_; // false if no velocity command has been specified
};



class RobotControllerPlugin : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
    RobotControllerPlugin();
    ~RobotControllerPlugin();

    bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    void stopping(const ros::Time& time);



    SensorMap &holdIn() {return _holdIn;}
    ControlMap &holdOut() {return _holdOut;}
    RobotDevice &device() {return *device_;}

    ros::NodeHandle node_;

    static const std::string LOG_PYTHON;

    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

	realtime_tools::RealtimeBuffer<Commands> command_;

	Commands command_struct_;

private:
    void fillSensors();
    void readControl(const ros::Time& time,const ros::Duration& period);

private:
    // python interactor methods
    void runPython(std::ostream &file,
                   const std::string &command,
                   bool print,
                   dynamicgraph::Interpreter &interpreter);

    virtual void startupPython();

    // sot sensor and control map
    SensorMap sensorsIn_;
    ControlMap controlValues_;

    //Hold or froze joint values for synchronization
    SensorMap _holdIn;
    ControlMap _holdOut;

    // Joint encoder and velocity vetors
    std::vector<double> joint_positionsOUT_;
    std::vector<double> joint_positionsIN_;
    std::vector<double> joint_velocityOUT_;
    std::vector<double> joint_velocityIN_;

    std::vector<double> error_raw;
    std::vector<double> error;
    std::vector<double> error_vel_last;
    std::vector<double> error_vel_sum;

    // Controller Parameters
    int loop_count_;
    ros::Time last_time_;
    //jointh
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<UrdfJointConstPtr> urdf_joints;

	hardware_interface::JointHandle joint_;
    std::vector<control_toolbox::Pid> pids_;

	hardware_interface::VelocityJointInterface *robot_;

    // ROS interface
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

    //ros::Publisher cmd_vel_pub_ ;

    //tf::TransformListener listener_;

    // SoT device
    RobotDevice *device_;

    double timeFromStart_;

    bool get_control;
    int count_loop;
};

}

#endif
