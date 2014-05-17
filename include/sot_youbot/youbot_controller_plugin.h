#ifndef YOUBOT_CONTROLLER_PLUGIN_H
#define YOUBOT_CONTROLLER_PLUGIN_H

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <sot_youbot/youbot_sot_controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sot_youbot/timer_utility.hh>
#include <sot_youbot/youbot_device.h>

using namespace timer;

namespace sot_youbot {

typedef boost::shared_ptr<pr2_mechanism_model::JointState> YoubotJointPtr;

class YoubotControllerPlugin : public pr2_controller_interface::Controller {
public:
    explicit YoubotControllerPlugin();
    virtual ~YoubotControllerPlugin();

    virtual bool init(pr2_mechanism_model::RobotState *robot,
                      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();

    ros::NodeHandle node_;


private:
    void fillSensors();
    void readControl();

private:
    // python interactor methods
    void runPython(std::ostream &file,
                   const std::string &command,
                   bool print,
                   dynamicgraph::Interpreter &interpreter);


    // sot sensor and control map
    SensorMap sensorsIn_;
    ControlMap controlValues_;

    // joint encoder and velocity vetors
    std::vector<double> joint_positionsOUT_;
    std::vector<double> joint_positionsIN_;
    std::vector<double> joint_velocityOUT_;
    std::vector<double> joint_velocityIN_;

    std::vector<double> error_raw;
    std::vector<double> error;
    Timer timer;


    // Pr2 Controller
    int loop_count_;
    ros::Time last_time_;
    std::vector<YoubotJointPtr> joints_;
    std::vector<control_toolbox::Pid> pids_;
    pr2_mechanism_model::RobotState *robot_;

    // ROS interface
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            control_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

    ros::Publisher cmd_vel_pub_, arm_vel_pub_ ;

    tf::TransformListener listener_;


    // SoT controller
    YoubotSotController sot_controller_;





    double timeFromStart_;


    bool get_control;
    int count_loop;
};

}

#endif
