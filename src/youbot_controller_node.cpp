#include "sot_youbot/youbot_controller_node.hh"
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <brics_actuator/JointPositions.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <stdlib.h> 


#include <dynamic_graph_bridge/ros_init.hh>

// boost thread variables
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

//boost units for unit assignment
//#include <boost/units/systems/si/plane_angle.hpp>

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>


namespace sot_youbot {

static const std::string JOINTNAME_PRE = "arm_joint_";

const std::string YoubotControllerNode::LOG_PYTHON="/tmp/youbot_sot_controller.out";
#define LOG_TRACE(x) sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " ) " << x << std::endl

// boost threading variables
boost::condition_variable cond;
boost::mutex mut;
bool data_ready;


void workThread(YoubotControllerNode *actl) {

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




std::ofstream logout;

YoubotControllerNode::YoubotControllerNode(ros::NodeHandle &n)
    : node_(n),
      device_("youBot_device"),
      loop_count_(0),
      get_control(true),
      count_loop(0)
{
    boost::thread thr(workThread,this);
    LOG_TRACE("");
    boost::unique_lock<boost::mutex> lock(mut);
    cond.wait(lock);
    startupPython();
    interpreter_->startRosService ();
    logout.open("/tmp/out.log", std::ios::out);
}

YoubotControllerNode::~YoubotControllerNode() {
}

void YoubotControllerNode::init() 
{

    // publishers and subscribers
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    /*
    arm_vel_1_pub_ = node_.advertise<std_msgs::Float64>("/arm_1/arm_joint_1/command",1);
    arm_vel_2_pub_ = node_.advertise<std_msgs::Float64>("/arm_1/arm_joint_2/command",1);
    arm_vel_3_pub_ = node_.advertise<std_msgs::Float64>("/arm_1/arm_joint_3/command",1);
    arm_vel_4_pub_ = node_.advertise<std_msgs::Float64>("/arm_1/arm_joint_4/command",1);
    arm_vel_5_pub_ = node_.advertise<std_msgs::Float64>("/arm_1/arm_joint_5/command",1);*/
    arm_vel_pub_ = node_.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command",100);
     

    js_subscriber =node_.subscribe("/joint_states", 1000, &YoubotControllerNode::js_callback,this);
    std::cout << "INIT" << std::endl;
    // base location tf listener
   
		try {
		 //listener_.waitForTransform("base_link","odom", ros::Time(0), ros::Duration(10.0));
		}
 catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
 }

    // Allocate space
    const unsigned int jsz = 7;
    
    joint_encoder_.resize(jsz);
    joint_velocity_.resize(jsz);


    std::cout << "INITEND" << std::endl;

}

void YoubotControllerNode::js_callback(const sensor_msgs::JointState &msg )
{
   for(int i = 0; i< 7;i++)
   {
     joint_encoder_[i] = msg.position[i+8];      
   }
}

void YoubotControllerNode::fillSensors() {
    // Get Arm Joint State
    sensorsIn_["joints"].setName("position");
    sensorsIn_["joints"].setValues(joint_encoder_);

    // Get Odometry
    tf::StampedTransform current_transform;
    listener_.waitForTransform("odom","base_link" ,ros::Time(0), ros::Duration(1.0));
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


// arm velocity
void YoubotControllerNode::publishArmVelocity(std::vector<double> &jv)
{
    brics_actuator::JointVelocities arm_vel_cmd;
    std::vector <brics_actuator::JointValue> armJointvels;
    armJointvels.resize(5);

    std::stringstream jointName;
 
    //std_msgs::Float64 msg[5];
   
    for (int i = 0; i < 5; i++)
    {
        
        
        jointName.str("");

        jointName << JOINTNAME_PRE << (i+1);

        armJointvels[i].joint_uri = jointName.str();

        armJointvels[i].value = jv[i];
        

        armJointvels[i].unit = "s^-1 rad";
        
        /*msg[i].data = jv[i]; */
        

     }
     /*
     arm_vel_1_pub_.publish(msg[0]);
     arm_vel_2_pub_.publish(msg[1]);
     arm_vel_3_pub_.publish(msg[2]);
     arm_vel_4_pub_.publish(msg[3]);
     arm_vel_5_pub_.publish(msg[4]);*/

    arm_vel_cmd.velocities = armJointvels;
    arm_vel_pub_.publish(arm_vel_cmd);

}

// base velocity
void YoubotControllerNode::publishBaseVelocity(std::vector<double> &baseVel)
{

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
    cmd_vel.linear.x = baseVel[0];
    cmd_vel.linear.y =baseVel[1];
    cmd_vel.linear.z =baseVel[2];
    cmd_vel.angular.x =baseVel[3];
    cmd_vel.angular.y =baseVel[4];
    cmd_vel.angular.z =baseVel[5];
    cmd_vel_pub_.publish(cmd_vel);

}

void YoubotControllerNode::publishBaseZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

void YoubotControllerNode::publishArmZeroVelocity(){

		for(int i=0;i<7;i++)
			joint_velocity_[i] = 0.0;

}



void YoubotControllerNode::readControl()
{

    double arm_velocity_measure, base_velocity_measure = 0;

    //  Arm controller
    joint_velocity_ = controlValues_["velocities"].getValues();

    //for(int i = 0; i < 5; i++)
    //   arm_velocity_measure += joint_velocity_[i];    


    // Base controller
    std::vector<double> base_velocity_ = controlValues_["ffvelocity"].getValues();
    base_velocity_measure = base_velocity_[0]+base_velocity_[0]+base_velocity_[5];  
    
    publishBaseVelocity(base_velocity_);
    publishArmVelocity(joint_velocity_);



}


void YoubotControllerNode::starting() 
{

		std::cout << "STARTING" << std::endl;
		//last_time_ = robot_->getTime();
		fillSensors();

		try {
				device_.setupSetSensors(sensorsIn_);
				device_.getControl(controlValues_);
		}
		catch (std::exception &e) {  throw e; }

		readControl();
    std::cout << "EnDSTARTING" << std::endl;

}

void YoubotControllerNode::update() 
{
		fillSensors();
		try {
		device_.nominalSetSensors(sensorsIn_);
		device_.getControl(controlValues_);
		}
		catch (std::exception &e) {throw e; }
		readControl();
}

void YoubotControllerNode::stopping() 
{
    std::cout << "STOPPING" << std::endl;
    publishBaseZeroVelocity();
    publishArmZeroVelocity();
}


/* ------------------------- python interactor methods ------------------------------*/
void YoubotControllerNode::startupPython() 
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
void YoubotControllerNode::runPython(std::ostream &file,
                            const std::string &command,bool print,
                            dynamicgraph::Interpreter &interpreter) 
{
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

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  Timer timer;

  sot_youbot::YoubotControllerNode ycn(n);
  ycn.init();
  ycn.starting();
  
  //Timer timer_cycle;
  //timer_cycle.start();
  while (ros::ok())
  {
    //timer_cycle.stop();
    ycn.update();
    ros::spinOnce();
    loop_rate.sleep();
   
    //ycn.publishArmZeroVelocity();
    //ycn.publishBaseZeroVelocity();
    //ros::spinOnce(); 
    //loop_rate_two.sleep();
    //timer.stop();
    
    //++count;
    //std::cout <<"time elapsed is "<<timer.getElapsedTimeInMilliSec()<< std::endl;
    //std::cout <<"time elapsed is "<<timer_cycle.getElapsedTimeInMilliSec()<< std::endl;
    //timer_cycle.start();
  }


  return 0;
}

