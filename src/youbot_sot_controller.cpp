#include "sot_youbot/youbot_sot_controller.h"
#include <pluginlib/class_list_macros.h>
#include <dynamic_graph_bridge/ros_init.hh>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

namespace sot_youbot {

const std::string YoubotSotController::LOG_PYTHON="/tmp/youbot_sot_controller.out";
#define LOG_TRACE(x) sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " ) " << x << std::endl

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void
workThread(YoubotSotController *actl) {
    dynamicgraph::Interpreter aLocalInterpreter(actl->node_);
    actl->interpreter_ = boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);

    std::cout << "Going through the thread." << std::endl;
    {
        boost::lock_guard<boost::mutex> lock(mut);
        data_ready=true;
    }
    cond.notify_all();
    ros::waitForShutdown();
}



YoubotSotController::YoubotSotController(std::string name)
: node_(dynamicgraph::rosInit(false,true))
, device_(name+"_device")
{
    std::cout << "Going through YoubotSotController." << std::endl;
    boost::thread thr(workThread,this);
    LOG_TRACE("");
    boost::unique_lock<boost::mutex> lock(mut);
    cond.wait(lock);
    startupPython();
    interpreter_->startRosService ();
}

YoubotSotController::~YoubotSotController() {
}

void
YoubotSotController::setupSetSensors(SensorMap &sensorsIn) {
    device_.setupSetSensors(sensorsIn);
}

void
YoubotSotController::nominalSetSensors(SensorMap &sensorsIn) {
    device_.nominalSetSensors(sensorsIn);
}

void
YoubotSotController::cleanupSetSensors(SensorMap &sensorsIn) {
    device_.cleanupSetSensors(sensorsIn);
}

void
YoubotSotController::getControl(ControlMap &controlOut) {


    try {
        LOG_TRACE("");
        device_.getControl(controlOut);
        LOG_TRACE("");
    }
    catch (dynamicgraph::sot::ExceptionAbstract &err) {
        LOG_TRACE(err.getStringMessage());
        throw err;
    }
}


void
YoubotSotController::runPython(std::ostream &file,
                            const std::string &command,bool print,
                            dynamicgraph::Interpreter &interpreter) {
    if (print == true){file << ">>> " << command << std::endl;} else{}
    std::string lerr(""),lout(""),lres("");
    interpreter.runCommand(command,lres,lout,lerr);
    if (lres != "None") {
        if (lres=="<NULL>") {
            file << lout << std::endl;
            file << "------" << std::endl;
            file << lerr << std::endl;

            std::string err("Exception catched during sot controller initialization, please check the log file: " + LOG_PYTHON);
            throw std::runtime_error(err);
        }
        else
            if (print == true){file << lres << std::endl;} else{}
    }
}





void
YoubotSotController::startupPython() {
    std::ofstream aof(LOG_PYTHON.c_str());
    runPython (aof, "import sys, os",true, *interpreter_);
    runPython (aof, "pythonpath = os.environ['PYTHONPATH']",true, *interpreter_);
    runPython (aof, "path = []",true, *interpreter_);
    runPython (aof, "for p in pythonpath.split(':'):\n"
                    "  if p not in sys.path:\n"
                    "    path.append(p)",true, *interpreter_);
    runPython (aof, "path.extend(sys.path)",true, *interpreter_);
    runPython (aof, "sys.path = path",true, *interpreter_);
    runPython (aof, "from dynamic_graph import plug",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.core import *",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.core import  SOT,FeaturePosition, Task",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.youbot.prologue import robot",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.dyninv import SolverKine",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits",true, *interpreter_);
    runPython (aof, "import dynamic_graph.sotcollision as sc",true, *interpreter_);

    runPython (aof, "a = sc.SotCollision('sc')",true, *interpreter_);
    runPython (aof, "dt = 0",true, *interpreter_);

    runPython (aof, "solver = SolverKine('sot_solver')",true, *interpreter_);
    runPython (aof, "solver.setSize (robot.dynamic.getDimension())",true, *interpreter_);
    runPython (aof, "robot.device.resize (robot.dynamic.getDimension())",true, *interpreter_);

    //runPython (aof, "plug (solver.control, robot.device.control)",true, *interpreter_);
    dynamicgraph::rosInit(true);

    aof.close();
}

}
