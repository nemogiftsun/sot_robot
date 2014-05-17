#include "sot_youbot/youbot_controller_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <brics_actuator/JointPositions.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <stdlib.h> 

#include <dynamic_graph_bridge/ros_init.hh>


#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

//boost units for unit assignment
//#include <boost/units/systems/si/plane_angle.hpp>


namespace sot_youbot {

static const std::string JOINTNAME_PRE = "arm_joint_";



std::ofstream logout;

YoubotControllerPlugin::YoubotControllerPlugin()
    : pr2_controller_interface::Controller(),
      sot_controller_("Pr2"),
      loop_count_(0),
      robot_(NULL),
      get_control(true),
      count_loop(0){


}

YoubotControllerPlugin::~YoubotControllerPlugin() {
}

bool
YoubotControllerPlugin::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {

    sot_controller_.node_ = n;
    cmd_vel_pub_ =  sot_controller_.node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //arm_vel_pub_ =  sot_controller_.node_.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command",1000);

    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }
    if (!robot->model_) {
        ROS_ERROR_STREAM("NULL model pointer");
        return false;
    }
 std::cout << "Initialized SSS!!" << std::endl;
    robot_ = robot;
        
    // Get the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!sot_controller_.node_.getParam("joints", joint_names)) {
        ROS_ERROR("No joints given. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Malformed joint specification. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    for (int i=0; i<joint_names.size(); ++i) {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Array of joint names should contain all strings. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
            return false;
        }
        YoubotJointPtr j;
        j.reset(robot->getJointState((std::string)name_value));
        if (!j) {
            ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name_value).c_str(), sot_controller_.node_.getNamespace().c_str());
            return false;
        }
        joints_.push_back(j);
    }

    // Ensures joints are calibrated
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!joints_[i]->calibrated_) {
            ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints_[i]->joint_->name.c_str(), sot_controller_.node_.getNamespace().c_str());
            return false;
        }
    }

 // Setup PID controllers
    std::string gains_ns;
    if (!sot_controller_.node_.getParam("gains", gains_ns))
        gains_ns = sot_controller_.node_.getNamespace() + "/gains";
    pids_.resize(joints_.size());
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name))) {
            if (!pids_[i].init(ros::NodeHandle(sot_controller_.node_,"pid_parameters"))) {
                ROS_ERROR("Failed to build PID controller");
                return false;
            }
        }
    }
    // TF Listener
    listener_.waitForTransform("base_link", "odom_combined", ros::Time(0), ros::Duration(1.0));

    // Allocate space
    const unsigned int jsz = joints_.size();
    
    joint_positionsOUT_.resize(jsz);
    joint_positionsIN_.resize(jsz);
    joint_velocityOUT_.resize(jsz);  
    joint_velocityIN_.resize(jsz);

    error_raw.resize(jsz);
    error.resize(jsz);

    controller_state_publisher_.reset(
        new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>
                (sot_controller_.node_, "state", 1));
    controller_state_publisher_->lock();
    for (size_t j=0; j<joints_.size(); ++j)
        controller_state_publisher_->msg_.joint_names.push_back(joints_[j]->joint_->name);
    controller_state_publisher_->msg_.desired.positions.resize(joints_.size());
    controller_state_publisher_->msg_.desired.velocities.resize(joints_.size());
    controller_state_publisher_->msg_.desired.accelerations.resize(joints_.size());
    controller_state_publisher_->msg_.actual.positions.resize(joints_.size());
    controller_state_publisher_->msg_.actual.velocities.resize(joints_.size());
    controller_state_publisher_->msg_.error.positions.resize(joints_.size());
    controller_state_publisher_->msg_.error.velocities.resize(joints_.size());
    controller_state_publisher_->unlock();

    timeFromStart_ = 0.0;

    std::cout << "Initialized!!!" << std::endl;


    return true;
}

void
YoubotControllerPlugin::fillSensors() {
    // Joint values/
    sensorsIn_["joints"].setName("position");
    //std::cout<<"[";
    for (unsigned int i=0; i<joints_.size(); ++i)
    {
        joint_positionsIN_[i] = joints_[i]->position_;
        //if(joints_[i]->joint_->limits)
           //std::cout<<joints_[i]->joint_->limits->upper<<",";
        //else
           //std::cout<<"no limit"<<",";
    }
    //std::cout<<"]"<<std::endl;
    sensorsIn_["joints"].setValues(joint_positionsIN_);

    // Joint velocities
    sensorsIn_["velocities"].setName("velocity");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_velocityIN_[i] = joints_[i]->velocity_;
    sensorsIn_["velocities"].setValues(joint_velocityIN_);

    // Get Odometry
    tf::StampedTransform current_transform;
    listener_.lookupTransform("odom_combined","base_link",ros::Time(0), current_transform);
    std::vector<double> odom(6);
    tf::Vector3 xyz = current_transform.getOrigin();
    tf::Quaternion q = current_transform.getRotation();
    odom[0] = xyz[0];
    odom[1] = xyz[1];
    odom[2] = 0.0;
    odom[3] = 0.0;
    odom[4] = 0.0;
    odom[5] = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));

    sensorsIn_["odometry"].setValues(odom);
 
}

void
YoubotControllerPlugin::readControl() {
    ros::Time time = robot_->getTime();
    ros::Duration dt = time - last_time_;
    last_time_ = time;

    //  Arm controller
    joint_positionsOUT_ = controlValues_["joints"].getValues();
    joint_velocityOUT_ = controlValues_["velocities"].getValues();
    
    // arm control
    for (unsigned int i=12; i<joints_.size(); ++i) {
        error[i] = joints_[i]->position_ - joint_positionsOUT_[i];

        double errord = joints_[i]->velocity_ - joint_velocityOUT_[i];

        joints_[i]->commanded_effort_ += pids_[i].updatePid(error[i], errord, dt);

    }

    //arm control
    /*
    brics_actuator::JointVelocities arm_vel_cmd;
    std::vector <brics_actuator::JointValue> armJointvels;
    armJointvels.resize(5);
    std::stringstream jointName;   
    for (int i = 0; i < 5; i++)
    {       
        jointName.str("");
        jointName << JOINTNAME_PRE << (i+1);
        armJointvels[i].joint_uri = jointName.str();
        armJointvels[i].value = joint_velocityOUT_[i];
        armJointvels[i].unit = "s^-1 rad";
    }
    arm_vel_cmd.velocities = armJointvels;
    arm_vel_pub_.publish(arm_vel_cmd);*/

    // Base controller
    geometry_msgs::Twist base_cmd;
    std::vector<double> vel = controlValues_["ffvelocity"].getValues();
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
    base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = vel[0];
    base_cmd.linear.y = vel[1];
    base_cmd.linear.z = vel[2];
    base_cmd.angular.x = vel[3];
    base_cmd.angular.y = vel[4];
    base_cmd.angular.z = vel[5];

    cmd_vel_pub_.publish(base_cmd);

    // State publishing
    if (loop_count_ % 10 == 0) {
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.header.stamp = time;
            for (size_t j=0; j<joints_.size(); ++j) {
                controller_state_publisher_->msg_.desired.positions[j] = joint_positionsOUT_[j];
                controller_state_publisher_->msg_.actual.positions[j] = joints_[j]->position_;
                controller_state_publisher_->msg_.actual.velocities[j] = joints_[j]->velocity_;
                controller_state_publisher_->msg_.actual.time_from_start= ros::Duration(timeFromStart_);
                controller_state_publisher_->msg_.error.positions[j] = error[j];
            }
            controller_state_publisher_->unlockAndPublish();
        }
    }
    ++loop_count_;
}


void
YoubotControllerPlugin::starting() {
		std::cout << "STARTING" << std::endl;
		fillSensors();

    try {
        sot_controller_.nominalSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    
		readControl();
  
    std::cout << "UPDATE CYCLE IN LOOP" << std::endl; 
    //timer.start();

}

void
YoubotControllerPlugin::update() {
        timer.start();

    
		fillSensors();
 
    try {
        sot_controller_.nominalSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }

		readControl();
    timer.stop();
    std::cout <<"time elapsed is "<<timer.getElapsedTimeInMilliSec()<< std::endl;

}

void
YoubotControllerPlugin::stopping() {
    std::cout << "STOPPING" << std::endl;
    // take care everything is destroyed
    // and robot freezes 

}

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(sot_youbot::YoubotControllerPlugin,
                       pr2_controller_interface::Controller)

}
