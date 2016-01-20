#ifndef ROBOT_CONTROLLER_PLUGIN_H
#define ROUBOT_CONTROLLER_PLUGIN_H

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sot_robot/sot_robot_device.h>


namespace sot_robot {

typedef boost::shared_ptr<pr2_mechanism_model::JointState> Pr2JointPtr;

class Pr2ControllerPlugin : public pr2_controller_interface::Controller {
public:
    explicit Pr2ControllerPlugin();
    virtual ~Pr2ControllerPlugin();

    virtual bool init(pr2_mechanism_model::RobotState *robot,
                      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();
    SensorMap &holdIn() {return _holdIn;}
    ControlMap &holdOut() {return _holdOut;}
    RobotDevice &device() {return device_;}
    ros::NodeHandle node_;
    static const std::string LOG_PYTHON;
    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

private:
    void fillSensors();
    void readControl();

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

    //hold or froze joint values for synchronization
    SensorMap _holdIn;
    ControlMap _holdOut;

    // joint encoder and velocity vetors
    std::vector<double> joint_positionsOUT_;
    std::vector<double> joint_positionsIN_;
    std::vector<double> joint_velocityOUT_;
    std::vector<double> joint_velocityIN_;

    std::vector<double> error_raw;
    std::vector<double> error;


    // Pr2 Controller
    int loop_count_;
    ros::Time last_time_;
    std::vector<Pr2JointPtr> joints_;
    std::vector<control_toolbox::Pid> pids_;
    pr2_mechanism_model::RobotState *robot_;

    // ROS interface
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            control_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

    ros::Publisher cmd_vel_pub_ ;

    tf::TransformListener listener_;

    // SoT device
    RobotDevice device_;

    double timeFromStart_;

    bool get_control;
    int count_loop;
};

}

#endif