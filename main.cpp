#include <cstdlib>
#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>


#include <string.h>
#include <Eigen/Dense>


#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>

class Module : public yarp::os::RFModule
{
public:
    bool configure(yarp::os::ResourceFinder &rf) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

private:
    double rate_;

    yarp::dev::PolyDriver drv_;
    yarp::dev::IPositionControl* p_control_;

    std::vector<std::string> joints_;
    std::vector<double> joints_speeds_original_;

    std::string class_name_ = "Mymodule";

};

int main(int argc, char** argv)
{
    const std::string module_name = "test";

    /* Check YARP network. */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << module_name + "::main(). Error: YARP network is not available.";
        return EXIT_FAILURE;
    }

    /* Load configuration file. */
    yarp::os::ResourceFinder rf;

    /* Initialize the module. */
    Module module;
    return module.runModule(rf);
}

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    rate_ = 100.0;

    /* Open the controlboardremapper device. */
    yarp::os::Property prop;
    prop.put("device", "remotecontrolboardremapper");

    /* Add joints list. */
    prop.addGroup("axesNames");
    yarp::os::Bottle &axes_names_bot = prop.findGroup("axesNames").addList();
    axes_names_bot.addString("torso_roll");
    axes_names_bot.addString("torso_pitch");
    axes_names_bot.addString("torso_yaw");
    axes_names_bot.addString("r_shoulder_pitch");
    axes_names_bot.addString("r_shoulder_roll");
    axes_names_bot.addString("r_shoulder_yaw");
    axes_names_bot.addString("r_elbow");
    axes_names_bot.addString("r_wrist_yaw");
    axes_names_bot.addString("r_wrist_roll");
    axes_names_bot.addString("r_wrist_pitch");

    /* Add remote control boards. */
    prop.addGroup("remoteControlBoards");
    yarp::os::Bottle &rcb_bot = prop.findGroup("remoteControlBoards").addList();
    rcb_bot.addString("/ergocubSim/torso");
    rcb_bot.addString("/ergocubSim/right_arm");

    /* Add local port to property. */
    prop.put("localPortPrefix", "/gb-ergocub-cartesian-controller/right_arm");

    /* Open the polydriver. */
    if (!drv_.open(prop))
    {
        yError() << class_name_ + "::configure(). Error: cannot open remotecontrolboardremapper .";
        return false;
    }

    if (!drv_.view(p_control_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IPositionControl.";
        return false;
    }

    if (!p_control_)
    {
        yError() << class_name_ + "::configure(). p_control_ not initialized.";
        return false;
    }

    yInfo() << class_name_ + "::configure(). Let's get ref speeds.";
    joints_speeds_original_.resize(axes_names_bot.size());

    if(!p_control_->getRefSpeeds(joints_speeds_original_.data()))
    {
        yError() << class_name_ + "::configure(). Cannot backup current joints speeds.";
        return false;
    }
    //--

    // It works this way.
    /*
    for (int i = 0; i < axes_names_bot.size(); ++i)
        if (!p_control_->getRefSpeed(i, i + joints_speeds_original_.data()))
        {
            yError() << class_name_ + "::configure(). Cannot backup current joints speeds.";
            return false;
        }
    */

    return true;
}

bool Module::close()
{
    return true;
}

double Module::getPeriod()
{
    return 1.0 / rate_;
}

bool Module::interruptModule()
{
    return true;
}

bool Module::updateModule()
{
    return true;
}
