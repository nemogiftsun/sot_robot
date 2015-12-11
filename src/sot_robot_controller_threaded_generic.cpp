#include "sot_robot/sot_robot_controller_threaded_generic.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <stdlib.h> 
#include "angles/angles.h"
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

//boost units for unit assignment
//#include <boost/units/systems/si/plane_angle.hpp>

namespace sot_robot {

static const std::string JOINTNAME_PRE = "arm_joint_";
//static const std::string ODOMFRAME = "odom";
static const std::string ODOMFRAME = "odom_combined";

const std::string RobotControllerPlugin::LOG_PYTHON="/tmp/robot_sot_controller.out";
#define LOG_TRACE(x) sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " ) " << x << std::endl


// boost threading variables
boost::condition_variable cond;
boost::condition cond2;
boost::condition cond3;
boost::mutex mut;
boost::mutex wait_start;
boost::mutex rmut, wmut;
bool data_ready;


boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
	boost::shared_ptr<urdf::Model> urdf(new urdf::Model);
	std::string urdf_str;
	// Check for robot_description in proper namespace
	if (nh.getParam(param_name, urdf_str))
	{
	if (!urdf->initString(urdf_str))
	{
	ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
	nh.getNamespace() << ").");
	return boost::shared_ptr<urdf::Model>();
	}
	}
	// Check for robot_description in root
	else if (!urdf->initParam("robot_description"))
	{
	ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
	return boost::shared_ptr<urdf::Model>();
	}
	return urdf;
}

// thread functions for sot control
void workThread(RobotControllerPlugin *actl) {

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

void sampleAndHold(RobotControllerPlugin *actl) {

    SensorMap deviceIn;
    ControlMap deviceOut;

    // Wait the start flag
    boost::unique_lock<boost::mutex> lock(wait_start);
    cond2.wait(lock);
	
    // Go go go !!!
    while (true) {
        {
            boost::mutex::scoped_lock lock(rmut);
            deviceIn = actl->holdIn();
        }
        actl->device().nominalSetSensors(deviceIn);
        try {
            LOG_TRACE("");
            actl->device().getControl(deviceOut);
            LOG_TRACE("");
        }
        catch (dynamicgraph::sot::ExceptionAbstract &err) {
            LOG_TRACE(err.getStringMessage());
            throw err;
        }
        actl->device().getControl(deviceOut);
        {
            boost::mutex::scoped_lock lock(wmut);
            actl->holdOut() = deviceOut;
        }
        cond3.notify_all();
        usleep(1);
    }
}


std::ofstream logout;


//Constructor
RobotControllerPlugin::RobotControllerPlugin()
    : loop_count_(0),
      robot_(NULL),
      get_control(true),
      count_loop(0){
			boost::thread thr(workThread,this);
			LOG_TRACE("");
			boost::unique_lock<boost::mutex> lock(mut);
			cond.wait(lock);
			startupPython();
			(*interpreter_).startRosService ();
			logout.open("/tmp/out.log", std::ios::out);
}


// Destructor
RobotControllerPlugin::~RobotControllerPlugin() {

}

bool RobotControllerPlugin::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {

	node_ = n;

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

	std::string device_name;
    if (!node_.getParam("robot", device_name)) {
        ROS_ERROR("No robot device name given. (namespace: %s)", node_.getNamespace().c_str());
        return false;
    }
	device_ = new RobotDevice(device_name);

    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }

    robot_ = robot;

    // Get the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam("joints", joint_names)) {
        ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
        return false;
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Malformed joint specification. (namespace: %s)", node_.getNamespace().c_str());
        return false;
    }

	// URDF joints
	boost::shared_ptr<urdf::Model> urdf = getUrdf(n, "robot_description");
	if (!urdf) {return false;}


/*
    for (int i=0; i<joint_names.size(); ++i) {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Array of joint names should contain all strings. (namespace: %s)", node_.getNamespace().c_str());
            return false;
        }
		RobotJointPtr j;
        j.reset(robot->getJointState((std::string)name_value));
        if (!j) {
            ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name_value).c_str(), node_.getNamespace().c_str());
            return false;
        }

        joints_.push_back(j);

    }

    // Ensures joints are calibrated
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!joints_[i]->calibrated_) {
            ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints_[i]->joint_->name.c_str(), node_.getNamespace().c_str());
            return false;
        }
    }

 // Setup PID controllers
    std::string gains_ns;
    if (!node_.getParam("gains", gains_ns))
        gains_ns = node_.getNamespace() + "/gains";
    pids_.resize(joints_.size());
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name))) {
            if (!pids_[i].init(ros::NodeHandle(node_,"pid_parameters"))) {
                ROS_ERROR("Failed to build PID controller");
                return false;
            }
        }
    }

    // TF Listener
    listener_.waitForTransform("base_link", ODOMFRAME, ros::Time(0), ros::Duration(1.0));

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
                (node_, "state", 1));
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

    // sample and hold thread initialized
    boost::thread sampleAndHoldProcess(sampleAndHold,this);

    std::cout << "Initialized!!!" << std::endl;
    last_time_ = robot->getTime();
    return true;
*/
}

void RobotControllerPlugin::fillSensors() {
    // Joint values/
    sensorsIn_["joints"].setName("position");
    for (unsigned int i=0; i<joints_.size(); ++i){
    	joint_positionsIN_[i] = joints_[i]->getPosition();
    }
    sensorsIn_["joints"].setValues(joint_positionsIN_);

    // Joint velocities
    sensorsIn_["velocities"].setName("velocity");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_velocityIN_[i] = joints_[i]->getVelocity();
    sensorsIn_["velocities"].setValues(joint_velocityIN_);

    // Get Odometry
    tf::StampedTransform current_transform;
    listener_.lookupTransform(ODOMFRAME,"base_link",ros::Time(0), current_transform);
    std::vector<double> odom(6);
    tf::Vector3 xyz = current_transform.getOrigin();
    tf::Quaternion q = current_transform.getRotation();
    odom[0] = xyz[0];
    odom[1] = xyz[1];
    odom[2] = 0;//joints_[0]->position_;
    odom[3] = 0.0;
    odom[4] = 0.0;
    odom[5] = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));

    sensorsIn_["odometry"].setValues(odom);
 
}

void RobotControllerPlugin::readControl() {

    //ros::Time time = robot_->getTime();
    //ros::Duration dt_ = time - last_time_;
    
    //  Arm controller
    joint_positionsOUT_ = controlValues_["joints"].getValues();
    joint_velocityOUT_ = controlValues_["velocities"].getValues();
    
    /* arm position control*/
/*
    for (unsigned int i=0; i<joints_.size(); ++i) {
        double errord = joints_[i]->velocity_ - joint_velocityOUT_[i];
    	if(joints_[i]->joint_->type == urdf::Joint::REVOLUTE)
        {
          angles::shortest_angular_distance_with_limits(joint_positionsOUT_[i], joints_[i]->position_, joints_[i]->joint_->limits->lower, joints_[i]->joint_->limits->upper,error[i]);

        }
        else if(joints_[i]->joint_->type == urdf::Joint::CONTINUOUS)
        {
          error[i] = angles::shortest_angular_distance(joint_positionsOUT_[i], joints_[i]->position_);
        }
        else //prismatic
        {
          error[i] = joints_[i]->position_ - joint_positionsOUT_[i];
        }

        joints_[i]->commanded_effort_ += pids_[i].updatePid(error[i], errord, dt_);
        }

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
    last_time_ = time;*/
}


void RobotControllerPlugin::starting() {

	std::cout << "STARTING" << std::endl;
	fillSensors();
    try {
			{
			  boost::mutex::scoped_lock lock(rmut);
			  _holdIn = sensorsIn_;
			}
		  	cond2.notify_all();
			{
				boost::mutex::scoped_lock lock(wmut);
				cond3.wait(lock);
				controlValues_ = _holdOut;
			}
		}
		catch (std::exception &e) {throw e; }
		readControl();
  
    std::cout << "UPDATE CYCLE IN LOOP" << std::endl; 
}

void RobotControllerPlugin::update(const ros::Time& time, const ros::Duration& period) {
    
		fillSensors();
 
    try {  
		  {
		      boost::mutex::scoped_lock lock(rmut);
		      _holdIn = sensorsIn_;
		  }
      cond2.notify_all();
		  {
		      boost::mutex::scoped_lock lock(wmut);
          if(_holdOut["joints"].getValues() == _holdOut["joints"].getValues())
          {
		        controlValues_ = _holdOut;
          }
		  }
		}
		catch (std::exception &e) {throw e; }

		readControl();
}

void RobotControllerPlugin::stopping() {
    std::cout << "STOPPING" << std::endl;
}


// python interactor services
void RobotControllerPlugin::startupPython() 
{
    std::ofstream aof(LOG_PYTHON.c_str());
    runPython (aof, "import sys, os",true, *interpreter_);
    runPython (aof, "pythonpath = os.environ['PYTHONPATH']",true, *interpreter_);  
    runPython (aof, "path = []",true, *interpreter_);
    runPython (aof, "for p in pythonpath.split(':'):\n"
                    "  if p not in sys.path:\n"
                    "    path.append(p)",true, *interpreter_);
    runPython (aof, "path.extend(sys.path)",true, *interpreter_);
    runPython (aof, "sys.path = path",true, *interpreter_);
    runPython (aof, "from dynamic_graph.sot.ur.prologue import sot",true, *interpreter_);

    dynamicgraph::rosInit(true);
    aof.close();
}

void RobotControllerPlugin::runPython(std::ostream &file,
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






/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS( sot_robot::RobotControllerPlugin, controller_interface::ControllerBase)


/*PLUGINLIB_DECLARE_CLASS(sot_robot,
                        RobotControllerPlugin,
                        sot_robot::RobotControllerPlugin,
                        controller_interface::ControllerBase)
*/
}
