#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"
#include "qm_controllers/GaitSwitch.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>
#include <ocs2_legged_robot_ros/gait/GaitKeyboardPublisher.h>

using namespace ocs2;
using namespace qm;
using namespace legged_robot;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaitSwitch");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    gait_mode_sub = nh.subscribe<std_msgs::String>("/gait_mode_cmd", 1, gaitCmdCallback);

    // Get gait command file
    std::string gaitCommandFile;
    nh.getParam("/gaitCommandFile", gaitCommandFile);
    myGaitKeyboardPublisher gaitCommand(nh, gaitCommandFile, "legged_robot", true);

    while (ros::ok())
    {
        if (is_converted)
        {
            gaitCommand.switchGaitMode(sub_gait_mode);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
