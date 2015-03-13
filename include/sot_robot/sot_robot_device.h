#ifndef ROBOT_DEVICE_H
#define ROBOT_DEVICE_H

#include <sot/core/device.hh>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot_robot/timer_utility.hh>

using namespace timer;

namespace sot_robot {

    typedef std::map<std::string, dynamicgraph::sot::SensorValues> SensorMap;
    typedef std::map<std::string, dynamicgraph::sot::ControlValues> ControlMap;

    class RobotDevice : public dynamicgraph::sot::Device {
        DYNAMIC_GRAPH_ENTITY_DECL();
    public:
        static const double TIMESTEP_DEFAULT;
        static const int NUMJOINTS;

    public:
        RobotDevice(const std::string &name);
        virtual ~RobotDevice();

        void setSensors(SensorMap &sensorsIn);
        void initSensors(SensorMap &sensorsIn);

        void setupSetSensors(SensorMap &sensorsIn);
        void nominalSetSensors(SensorMap &sensorsIn);
        void cleanupSetSensors(SensorMap &sensorsIn);
        void findControl();
        void getControl(ControlMap &controlOut);

    protected:
        void updateRobotState(const std::vector<double> &anglesIn);

    protected:
        double timestep_;
        dynamicgraph::Vector previous_state_;
        //dynamicgraph::Signal<dynamicgraph::Vector, int> robotState_;

    private:
        //dynamicgraph::Vector mlRobotState;
        dynamicgraph::sot::MatrixRotation pose;
        std::vector<double> baseff_;
        Timer timer;
        int loop_count_;
        bool init_required;
    };
}

#endif
