#include "sot_youbot/youbot_device.h"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

namespace sot_youbot {

const double YoubotDevice::TIMESTEP_DEFAULT = 0.05;

YoubotDevice::YoubotDevice(const std::string &name)
: dynamicgraph::sot::Device(name),
  timestep_(TIMESTEP_DEFAULT),
  previous_state_(),
  //robotState_ ("StackOfTasks(" + name + ")::output(vector)::robotState"),
  pose(),
  baseff_(),
  loop_count_(0)
{
    sotDEBUGIN(25);
    //signalRegistration(robotState_);
    baseff_.resize(12);

    std::string docstring;
    docstring =
       "\n"
       "    Integrate dynamics for time step provided as input\n"
       "\n"
       "      take one floating point number as input\n"
       "\n";
     addCommand("increment",
            dynamicgraph::command::makeCommandVoid1((Device&)*this,
                     &Device::increment, docstring));

     sotDEBUGOUT(25);
}

YoubotDevice::~YoubotDevice() {
}

void
YoubotDevice::setSensors(SensorMap &sensorsIn) {
    sotDEBUGIN(25);
    SensorMap::iterator it;

    /* Joint Posiitons
    
    it = sensorsIn.find("joints");
    if (it != sensorsIn.end()) {
        std::vector<double> anglesIn = it->second.getValues();
        try {
        for (unsigned i=0;i<6; ++i)
            state_(i) = 0.;
            for (unsigned i=0; i<45; ++i)
             {
                state_(i+6) = anglesIn[i];              
             }
        }
        catch (...) {}
    }*/

    // Joint velocity
    it = sensorsIn.find("velocities");
    if (it != sensorsIn.end()) {
        std::vector<double> velIn = it->second.getValues();
        try {
        for (unsigned i=0;i<6; ++i)
            velocity_(i) = 0.;
            for (unsigned i=0; i<45; ++i)
             {
                velocity_(i+6) = velIn[i];              
             }
        }
        catch (...) {}
    }


    // Odometry
    it = sensorsIn.find("odometry");
    if (it != sensorsIn.end()) {
        std::vector<double> odomIn = it->second.getValues();
        try {
            for (unsigned i=0; i<6; ++i)
                state_(i) = odomIn[i];
        }
        catch (...) {}
    }

    sotDEBUGOUT(25);
}

void
YoubotDevice::setupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
YoubotDevice::nominalSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
YoubotDevice::cleanupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
YoubotDevice::findControl() {
    sotDEBUGIN(25);
    std::vector<double> anglesOut;
    anglesOut.resize(state_.size());
    std::vector<double> velocitiesOut;
    velocitiesOut.resize(state_.size());

    try { increment(timestep_); }
    catch (...) {
        //std::cout << "Increment error (" << loop_count_ << ") (" << controlSIN << ")" << std::endl;
    }
    //previous_state_ = state_;
}

void
YoubotDevice::getControl(ControlMap &controlOut) {

    sotDEBUGIN(25);
    std::vector<double> anglesOut;
    anglesOut.resize(state_.size());
    std::vector<double> velocitiesOut;
    velocitiesOut.resize(state_.size());
    int time = stateSOUT.getTime();

    try { increment(timestep_); }
    catch (...) {
        //std::cout << "Increment error (" << loop_count_ << ") (" << controlSIN << ")" << std::endl;
    }
    //previous_state_ = state_;

     //++loop_count_;

    //sotDEBUG(25) << "state = " << state_ << std::endl;
    //sotDEBUG(25) << "diff = " << ((previous_state_.size() == state_.size()) ?
    //                                  (state_ - previous_state_) : state_ ) << std::endl;


    // Get control
    /*
   stateSOUT .setConstant(state_); stateSOUT.setTime(time);
  try
    {
      periodicCallBefore_.run(time+1);
    }
  catch (std::exception& e)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< e.what () << std::endl;
    }
  catch (const char* str)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< str << std::endl;
    }
  catch (...)
    {
      std::cerr
	<< "unknown exception caught while"
	<< " running periodical commands (before)" << std::endl;
    }*/

    ml::Vector control;
    try {
          
          controlSIN( time );
          control = controlSIN.accessCopy();
    }
    catch (...) {
        control.resize(state_.size());
        for (unsigned i=0; i<state_.size(); ++i)
            control(i) = 0.;
    }
  /*
  try
    {
      periodicCallAfter_.run(time+1);
    }
  catch (std::exception& e)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< e.what () << std::endl;
    }
  catch (const char* str)
    {
      std::cerr
	<< "exception caught while running periodical commands (before): "
	<< str << std::endl;
    }
  catch (...)
    {
      std::cerr
	<< "unknown exception caught while"
	<< " running periodical commands (before)" << std::endl;
    }*/
    // Specify joint values
    if (anglesOut.size() != state_.size() - 6)
        anglesOut.resize(state_.size() - 6);
    for (unsigned int i=6; i<state_.size(); ++i)
        anglesOut[i-6] = state_(i);
    controlOut["joints"].setValues(anglesOut);

    // Specify joint velocity
    if (velocitiesOut.size() != state_.size() - 6)
        velocitiesOut.resize(state_.size() - 6);
    for (unsigned int i=6; i<state_.size(); ++i)
        velocitiesOut[i-6] = control(i);
    controlOut["velocities"].setValues(velocitiesOut);

    // Update position of free flyer
    for (int i = 0;i < 3; ++i)
        baseff_[i*4+3] = freeFlyerPose () (i, 3);
    for(unsigned i = 0;i < 3; ++i)
        for(unsigned j = 0; j < 3; ++j)
            baseff_[i * 4 + j] = freeFlyerPose () (i, j);
    controlOut["baseff"].setValues(baseff_);

    // Update velocity of free flyer
    std::vector<double> ffvelocity(6,0.);
    for (int i=0; i<6; ++i)
        ffvelocity[i] = control(i);

    controlOut["ffvelocity"].setValues(ffvelocity);

    sotDEBUGOUT(25);
}

void
YoubotDevice::updateRobotState(const std::vector<double> &anglesIn)
{
    sotDEBUGIN(25);
    for (unsigned i=0; i<anglesIn.size(); ++i)
        state_(i+6) = anglesIn[i];
        //mlRobotState(i+6) = anglesIn[i];
        //}
    //robotState_.setConstant(mlRobotState);
    sotDEBUGOUT(25);
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(YoubotDevice,"YoubotDevice");

}
