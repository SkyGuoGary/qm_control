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
    if (argc != 2)
    {
        ROS_ERROR("move order");
        return 0;
    }
    int param = std::stoi(argv[1]);

    ros::NodeHandle nh;
    marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    ros::Rate loop_rate(f);

    initial_pose = setInitPose(); // only initailized, not real initail pose
    marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);
    marker_vel_sub = nh.subscribe<geometry_msgs::Twist>("/marker_cmd_vel", 1, markerVelCallback);

    // waiting for initial ee pose
    while (!received_curr_marker && ros::ok() && ros::master::check())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    visualization_msgs::InteractiveMarkerFeedback marker_pose;
    marker_pose = initial_pose;
    visualization_msgs::InteractiveMarkerFeedback center_point;
    center_point.pose.position.x = 0.55;
    center_point.pose.position.y = 0.175;
    center_point.pose.position.z = 0.707;
    center_point.pose.orientation.x = 0.0;
    center_point.pose.orientation.y = sqrt(0.5);
    center_point.pose.orientation.z = 0.0;
    center_point.pose.orientation.w = sqrt(0.5);
    while (ros::ok())
    {
        double step_time = 1.0 / f;
        //subscribe pos_vel from /marker_cmd_vel
        tf2::Vector3 pos_vel_(marker_vel.linear.x, marker_vel.linear.y, marker_vel.linear.z);
        geometry_msgs::Vector3 pos_vel=tf2::toMsg(pos_vel_);

        // markerPoseVelControl(marker_pose, step_time, pos_vel);

        geometry_msgs::Vector3 target_marker;
        target_marker.x = 2;
        target_marker.y = 2;
        target_marker.z = 1;
        markerPosePosControl(marker_pose, target_marker, step_time, 0.1, param);
        
            

        // tf2::Vector3 center_(0.7, 0, 0.7);
        // geometry_msgs::Vector3 center = tf2::toMsg(center_);

        // tf2::Quaternion quat_(0, 1/sqrt(2), 0, 1/sqrt(2)); // rotate 90 degree around world_Y
        // // tf2::Quaternion quat_(0, 0, 0, 1); // translate origin only
        // quat_.normalize();
        // geometry_msgs::Quaternion center_frame_quat = tf2::toMsg(quat_);
        
        // double omega = 0.1;
        // double desired_angle = 0.5*M_PI;
        // std::string axis=argv[1];

        // markerPoseAngularVelControl(marker_pose, center,
        //                             step_time, omega, argv[1], param);

        // if (is_rotated)
        // {
        //     // use radian instead of degree
        //     markerPoseAngularPosControl(
        //         marker_pose, center, center_frame_quat, 
        //         step_time, omega, axis, desired_angle, is_rotated, param);
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
