#ifndef YOUBOT_CONTROLLER_NODE_H
#define YOUBOT_CONTROLLER_NODE_H


#include <sot_youbot/youbot_device.h>
#include <sot_youbot/timer_utility.hh>

#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/abstract-sot-external-interface.hh>

using namespace timer;

namespace sot_youbot {


class YoubotControllerNode {
public:
    explicit YoubotControllerNode(ros::NodeHandle &n);
    virtual ~YoubotControllerNode();

    void init();
    void starting();
    void update();
    void stopping();
    void publishArmZeroVelocity();
    void publishBaseZeroVelocity();

    SensorMap &holdIn() {return _holdIn;}
    ControlMap &holdOut() {return _holdOut;}
    YoubotDevice &device() {return device_;}

private:
    // sensor and controller methods
    void fillSensors();
    void readControl();

    void publishBaseVelocity(std::vector<double> &baseVel);
    void publishArmVelocity(std::vector<double> &jv);
    void publishArmPositions(std::vector<double> &jp);
    // python interactor methods
    void runPython(std::ostream &file,
                   const std::string &command,
                   bool print,
                   dynamicgraph::Interpreter &interpreter);

    virtual void startupPython();
       
    
public:

    static const std::string LOG_PYTHON;
    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;
    // ROS node
    ros::NodeHandle node_;

private:

    int loop_count_;
    ros::Time last_time_;

    // SoT device
    YoubotDevice device_;

    // sot sensor and control map
    SensorMap sensorsIn_;
    ControlMap controlValues_;

    //hold or froze joint values for synchronization
    SensorMap _holdIn;
    ControlMap _holdOut;

    // joint encoder and velocity vetors
    std::vector<double> joint_encoder_;
    std::vector<double> joint_velocityin_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_positions_;
   

    Timer timer;




    
    // joint state subscriber
    ros::Subscriber js_subscriber;
    void js_callback(const sensor_msgs::JointState &msg);
    
    // cmd vel publisher am
    ros::Publisher cmd_vel_pub_, arm_vel_pub_ ;
    ros::Publisher arm_jp_pub_;

    // transform listener
    tf::TransformListener listener_;

    double timeFromStart_;

    bool get_control;
    int count_loop;
};

}

#endif
