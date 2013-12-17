#ifndef YOUBOT_SOT_CONTROLLER_H
#define YOUBOT_SOT_CONTROLLER_H

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <sot_youbot/pr2_device.h>
#include <dynamic_graph_bridge/ros_interpreter.hh>

namespace sot_youbot {

class YoubotSotController : public pr2_controller_interface::Controller
{
public:
    explicit YoubotSotController();

    virtual bool init(pr2_mechanism_model::RobotState *robot,
                      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();

private:
    YoubotDevice *device_;
    jointMap_t joint_map_;
    dynamicgraph::Interpreter interpreter_;

    static const std::string LOG_PYTHON;
    void runPython(std::ostream& file, const std::string& command);
};

}

#endif
