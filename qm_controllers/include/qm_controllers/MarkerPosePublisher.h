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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

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
    init_pose.pose.orientation.y = sqrt(0.5);
    init_pose.pose.orientation.z = 0.0;
    init_pose.pose.orientation.w = sqrt(0.5);
    return init_pose;
}
// position vel control
void markerPoseVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    double step_time, double v_x, double v_y, double v_z)
{
    curr.pose.position.x += v_x * step_time;
    curr.pose.position.y += v_y * step_time;
    curr.pose.position.z += v_z * step_time;
    marker_pose_pub.publish(curr);
}

// angular position control
double roll, pitch, yaw;
void markerPoseAngularPosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    double step_time, double angular_velocity, std::string rotation_axis,
    bool rotate_orientation = 1)
{
    // check if marker have reached target angle
    // double current_angle = 2.0 * acos(curr.pose.orientation.w);
    // if (fabs(current_angle) >= fabs(init_angle+rotation_angle))
    // {
    //     ROS_INFO("Reached specified rotation angle!");
    //     is_reached=1;
    //     return;
    // }

    // 计算旋转增量
    double step_angle = angular_velocity * step_time;

    geometry_msgs::Transform current_pose;
    double x = curr.pose.position.x;
    double y = curr.pose.position.y;
    double z = curr.pose.position.z;
    current_pose.rotation = curr.pose.orientation;

    // 更新xyz坐标
    if (rotation_axis == "x")
    {
        // 绕X轴旋转
        double new_y = y * cos(step_angle) - z * sin(step_angle);
        double new_z = y * sin(step_angle) + z * cos(step_angle);
        y = new_y;
        z = new_z;
    }
    else if (rotation_axis == "y")
    {
        // 绕Y轴旋转
        double new_x = x * cos(step_angle) + z * sin(step_angle);
        double new_z = -x * sin(step_angle) + z * cos(step_angle);
        x = new_x;
        z = new_z;
    }
    else if (rotation_axis == "z")
    {
        // 绕Z轴旋转
        double new_x = x * cos(step_angle) - y * sin(step_angle);
        double new_y = x * sin(step_angle) + y * cos(step_angle);
        x = new_x;
        y = new_y;
    }
    // if you want to rotate ee orientation together
    if (rotate_orientation)
    {
        // 转为tf的四元数
        tf2::Quaternion rotation_quaternion;
        tf2::fromMsg(current_pose.rotation, rotation_quaternion);

        // // set rotation axis vector
        // tf2::Vector3 axis;
        // if (rotation_axis == "x")
        //     axis.setValue(1.0, 0.0, 0.0);
        // if (rotation_axis == "y")
        //     axis.setValue(0.0, 1.0, 0.0);
        // if (rotation_axis == "z")
        //     axis.setValue(0.0, 0.0, 1.0);

        // // 构建绕点旋转的变换矩阵
        // tf2::Matrix3x3 rotation_matrix(rotation_quaternion);
        // // 计算当前旋转轴在世界坐标系中的方向
        // tf2::Vector3 world_axis = rotation_matrix * axis;

        // // 更新四元数
        // tf2::Quaternion rotation_increment(world_axis, step_angle);
        // rotation_quaternion = rotation_quaternion * rotation_increment;

        // // 将更新后的四元数转换回geometry_msgs::Quaternion
        // current_pose.rotation = tf2::toMsg(rotation_quaternion);

        tf2::Matrix3x3(rotation_quaternion).getRPY(roll, pitch, yaw);
        if (rotation_axis == "x")
            roll += step_angle;
        if (rotation_axis == "y")
            pitch += step_angle;
        if (rotation_axis == "z")
            yaw += step_angle;
        rotation_quaternion.setRPY(roll, pitch, yaw);
        std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;
        current_pose.rotation = tf2::toMsg(rotation_quaternion);
    }

    curr.pose.position.x = x;
    curr.pose.position.y = y;
    curr.pose.position.z = z;
    curr.pose.orientation = current_pose.rotation;

    std::cout << "position:" << curr.pose.position << std::endl;
    marker_pose_pub.publish(curr);
}