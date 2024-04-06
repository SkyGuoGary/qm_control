#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"
#include "qm_controllers/MarkerPosePublisher.h"

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
    ros::init(argc, argv, "markerPosePub");

    ros::NodeHandle nh;
    marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    ros::Rate loop_rate(f);

    initial_pose = setInitPose();
    marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);
    marker_vel_sub = nh.subscribe<geometry_msgs::Twist>("/marker_cmd_vel", 1, markerVelCallback);

    // waiting for initial ee pose
    while (!received_curr && ros::ok() && ros::master::check())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    visualization_msgs::InteractiveMarkerFeedback marker_pose;
    marker_pose = initial_pose;

    while (ros::ok())
    {
        //update marker_pose
        pose_x_vel = marker_vel.linear.x;
        pose_y_vel = marker_vel.linear.y;
        pose_z_vel = marker_vel.linear.z;
        marker_pose = markerPoseVelControl(marker_pose, 1.0 / f, pose_x_vel, pose_y_vel, pose_z_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
