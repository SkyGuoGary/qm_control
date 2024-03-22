#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"
#include "qm_controllers/MarkerPosePublisher.h"


ros::Publisher marker_pose_pub;
ros::Subscriber marker_feedback_sub, marker_vel_sub;

const int f = 10;
// position delta between obs & truth
const float delta_x = -0.003;
const float delta_y = -0.0015;
const float delta_z = 0.011;
// velocity control
geometry_msgs::Twist marker_vel;
double pose_x_vel = 0, pose_y_vel = 0, pose_z_vel = 0;
visualization_msgs::InteractiveMarkerFeedback initial_pose;
visualization_msgs::InteractiveMarkerFeedback marker_pose;
bool received_curr = 0;

void eeStateCallback(const qm_msgs::ee_state::ConstPtr &obs)
{
    initial_pose.pose.position.x = obs->state[0] + delta_x;
    initial_pose.pose.position.y = obs->state[1] + delta_y;
    initial_pose.pose.position.z = obs->state[2] + delta_z;

    initial_pose.pose.orientation.x = obs->state[3];
    initial_pose.pose.orientation.y = obs->state[4];
    initial_pose.pose.orientation.z = obs->state[5];
    initial_pose.pose.orientation.w = obs->state[6];

    received_curr = 1;
    
}
void markerVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    marker_vel = *msg;
}

visualization_msgs::InteractiveMarkerFeedback setInitPose()
{
    visualization_msgs::InteractiveMarkerFeedback init_pose;
    init_pose.pose.position.x = 0.55;
    init_pose.pose.position.y = 0.175;
    init_pose.pose.position.z = 0.707;
    init_pose.pose.orientation.x = 0.0;
    init_pose.pose.orientation.y = 0.707;
    init_pose.pose.orientation.z = 0.0;
    init_pose.pose.orientation.w = 0.707;
    return init_pose;
}

visualization_msgs::InteractiveMarkerFeedback markerPoseVelControl(
    visualization_msgs::InteractiveMarkerFeedback curr,
    double step_time, double v_x, double v_y, double v_z)
{
    curr.pose.position.x += v_x * step_time;
    curr.pose.position.y += v_y * step_time;
    curr.pose.position.z += v_z * step_time;
    marker_pose_pub.publish(curr);
    // ROS_INFO("marker_x = %lf", curr.pose.position.x);
    // ROS_INFO("marker_y = %lf", curr.pose.position.y);
    // ROS_INFO("marker_z = %lf", curr.pose.position.z);
    return curr;
    
}