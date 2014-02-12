#include "sot_youbot/youbot_controller_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <std_msgs/Bool.h>
#include <stdlib.h> 


//boost units for unit assignment
//#include <boost/units/systems/si/plane_angle.hpp>


namespace sot_youbot {

static const std::string JOINTNAME_PRE = "arm_joint_";

std::ofstream logout;

YoubotControllerPlugin::YoubotControllerPlugin()
    : pr2_controller_interface::Controller(),
      sot_controller_("youBot"),
      loop_count_(0),
      robot_(NULL) {
    logout.open("/tmp/out.log", std::ios::out);
}

YoubotControllerPlugin::~YoubotControllerPlugin() {
}

bool
YoubotControllerPlugin::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {
    sot_controller_.node_ = n;
    cmd_vel_pub_ = sot_controller_.node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    arm_vel_pub_ = sot_controller_.node_.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command",1);
    controller_status_pub_ = sot_controller_.node_.advertise<std_msgs::Bool>("/sot_youbot/controller_status",1);
    jointstate_sub =sot_controller_.node_.subscribe("/joint_states", 1000, &YoubotControllerPlugin::jointstatesub_cb,this);
    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }
    if (!robot->model_) {
        ROS_ERROR_STREAM("NULL model pointer");
        return false;
    }
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

    // TF Listener
    listener_.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

    // Allocate space
    const unsigned int jsz = joints_.size();
    joint_encoder_.resize(jsz);
    joint_velocity_.resize(jsz);
    joint_control_.resize(jsz);
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

    return true;
}

void YoubotControllerPlugin::jointstatesub_cb(const sensor_msgs::JointState &msg )
{
 
   for(int i = 8; i< 13;i++)
   {
     //joints_[i]->position_ = msg.position[i];
   } 
}

void
YoubotControllerPlugin::fillSensors() {
    // Joint values
    sensorsIn_["joints"].setName("position");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_encoder_[i] = joints_[i]->position_;
    sensorsIn_["joints"].setValues(joint_encoder_);

    // Get Odometry
    tf::StampedTransform current_transform;
    listener_.lookupTransform("odom","base_link",ros::Time(0), current_transform);
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
    brics_actuator::JointVelocities arm_vel_cmd;
    joint_control_ = controlValues_["joints"].getValues();
    joint_velocity_ = controlValues_["velocities"].getValues();
    std::stringstream jointName;
    std::vector <brics_actuator::JointValue> armJointvels;
    armJointvels.resize(5);
    double arm_abs_vel = 0;
    for (int i = 0; i < 5; i++)
    {
        jointName.str("");

        jointName << JOINTNAME_PRE << (i+1);

        armJointvels[i].joint_uri = jointName.str();

        armJointvels[i].value = joint_velocity_[i];

        armJointvels[i].unit = "s^-1 rad";
   
        arm_abs_vel += armJointvels[i].value;

     }
    arm_vel_cmd.velocities = armJointvels;
    arm_vel_pub_.publish(arm_vel_cmd);

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
                controller_state_publisher_->msg_.desired.positions[j] = joint_control_[j];
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
    last_time_ = robot_->getTime();

    fillSensors();
    try {
        sot_controller_.setupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();

}

void
YoubotControllerPlugin::update() {
    fillSensors();
    try {
        sot_controller_.nominalSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();
}

void
YoubotControllerPlugin::stopping() {
    std::cout << "STOPPING" << std::endl;

    /*fillSensors();
    try {
        sot_controller_.cleanupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();*/
}



/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(sot_youbot::YoubotControllerPlugin,
                       pr2_controller_interface::Controller)

}
