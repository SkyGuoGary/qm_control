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

// compute (x,y,z) of marker pose after rotation
tf2::Vector3 translationRotate(const visualization_msgs::InteractiveMarkerFeedback relative_position,
                               double step_angle, std::string rotation_axis)
{
    double x = relative_position.pose.position.x;
    double y = relative_position.pose.position.y;
    double z = relative_position.pose.position.z;

    // update translation part
    if (rotation_axis == "x")
    {
        double new_y = y * cos(step_angle) - z * sin(step_angle);
        double new_z = y * sin(step_angle) + z * cos(step_angle);
        y = new_y;
        z = new_z;
    }
    else if (rotation_axis == "y")
    {
        double new_x = x * cos(step_angle) + z * sin(step_angle);
        double new_z = (-x) * sin(step_angle) + z * cos(step_angle);
        x = new_x;
        z = new_z;
    }
    else if (rotation_axis == "z")
    {
        double new_x = x * cos(step_angle) - y * sin(step_angle);
        double new_y = x * sin(step_angle) + y * cos(step_angle);
        x = new_x;
        y = new_y;
    }
    std::cout << "relative position: " << std::endl
              << relative_position.pose.position;

    tf2::Vector3 res_point(x, y, z);
    return res_point;
}

// compute (qx,qy,qz,qw) of marker pose after rotation, without center frame rotation
tf2::Quaternion quatRotate(const visualization_msgs::InteractiveMarkerFeedback curr,
                           const geometry_msgs::Vector3 center_point,
                           double step_angle, std::string rotation_axis)
{
    // calculate current marker's tf in world
    tf2::Vector3 translation_of_curr(curr.pose.position.x, curr.pose.position.y, curr.pose.position.z);
    tf2::Quaternion curr_quat_inW(curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w);
    tf2::Transform tf_of_curr_inW(curr_quat_inW, translation_of_curr);

    // center frame's origin in world
    tf2::Vector3 translation_of_C_O;
    tf2::fromMsg(center_point, translation_of_C_O);
    // center frame's quat in world, default is as same as world
    tf2::Quaternion quat_of_CinW(0, 0, 0, 1);
    // center frame's tf in world
    tf2::Transform tf_of_CinW(quat_of_CinW, translation_of_C_O);

    // calculate current marker's tf in center frame
    tf2::Transform tf_of_curr_inC = tf_of_CinW.inverse() * tf_of_curr_inW;

    // rotate marker according to RPY of curr in center frame
    tf2::Quaternion tmp = tf_of_curr_inC.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(tmp).getRPY(roll, pitch, yaw);
    if (rotation_axis == "x")
        roll += step_angle;
    else if (rotation_axis == "y")
        pitch += step_angle;
    else if (rotation_axis == "z")
        yaw += step_angle;
    else
        ROS_ERROR("NO AXIS SPECIFIED!");
    std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;

    // change tmp
    tmp.setRPY(roll, pitch, yaw);
    // new marker's tf in center frame
    tf2::Transform tf_of_new_inC;
    tf_of_new_inC.setRotation(tmp);
    // calculate new marker's tf in world
    tf2::Transform tf_of_new_inW = tf_of_CinW * tf_of_new_inC;
    return tf_of_new_inW.getRotation();
}

// compute (x,y,z,qx,qy,qz,qw) of marker pose after rotation on any axis
tf2::Transform poseRotate(const visualization_msgs::InteractiveMarkerFeedback curr,
                          const geometry_msgs::Vector3 center_point,
                          const geometry_msgs::Quaternion center_frame_quat,
                          double step_angle, std::string rotation_axis)
{
    // get current marker's tf in world
    tf2::Vector3 translation_of_curr(curr.pose.position.x, curr.pose.position.y, curr.pose.position.z);
    tf2::Quaternion curr_quat_inW(curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w);
    tf2::Transform tf_of_curr_inW(curr_quat_inW, translation_of_curr);

    // center frame's origin in world
    tf2::Vector3 translation_of_C_O;
    tf2::fromMsg(center_point, translation_of_C_O);
    // center frame's quat in world, default is as same as world
    tf2::Quaternion quat_of_CinW;
    tf2::fromMsg(center_frame_quat, quat_of_CinW);
    // center frame's tf in world
    tf2::Transform tf_of_CinW(quat_of_CinW, translation_of_C_O);

    // calculate current marker's tf in center frame
    tf2::Transform tf_of_curr_inC = tf_of_CinW.inverse() * tf_of_curr_inW;

    // translation
    visualization_msgs::InteractiveMarkerFeedback curr_inC;
    curr_inC.pose.position.x = tf_of_curr_inC.getOrigin().x();
    curr_inC.pose.position.y = tf_of_curr_inC.getOrigin().y();
    curr_inC.pose.position.z = tf_of_curr_inC.getOrigin().z();
    tf2::Vector3 new_trans_inC = translationRotate(curr_inC, step_angle, rotation_axis);

    tf2::Transform tf_of_new_inC, tf_of_new_inW;
    tf_of_new_inC.setOrigin(new_trans_inC);
    // calculate new translation after rotation
    tf_of_new_inW = tf_of_CinW * tf_of_new_inC;
    tf2::Vector3 trans_of_new_inW = tf_of_new_inW.getOrigin();

    // rotate marker according to RPY of curr in center frame
    tf2::Quaternion tmp = tf_of_curr_inC.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(tmp).getRPY(roll, pitch, yaw);
    if (rotation_axis == "x")
        roll += step_angle;
    else if (rotation_axis == "y")
        pitch += step_angle;
    else if (rotation_axis == "z")
        yaw += step_angle;
    else
        ROS_ERROR("NO AXIS SPECIFIED!");
    std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;

    tmp.setRPY(roll, pitch, yaw);
    // new marker's tf in center frame
    tf_of_new_inC.setRotation(tmp);
    // calculate new marker's tf in world
    tf_of_new_inW = tf_of_CinW * tf_of_new_inC;
    
    // set new marker's translation
    tf_of_new_inW.setOrigin(trans_of_new_inW);
    return tf_of_new_inW;
}

// angular velocity control, won't stop, with center_frame_quat
void markerPoseAngularVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const geometry_msgs::Quaternion center_frame_quat,
    double step_time, double angular_velocity, std::string rotation_axis,
    bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    tf2::Vector3 res_translation;
    tf2::Quaternion res_quat;
    tf2::Transform res_tf;
    // rotate (x,y,z) and (qx,qy,qz,qw) together
    res_tf = poseRotate(curr, center_point, center_frame_quat, step_angle, rotation_axis);
    res_translation = res_tf.getOrigin();

    // if you want to rotate marker orientation
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = res_tf.getRotation();
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}

// angular velocity control, won't stop, WITHOUT center_frame rotation
void markerPoseAngularVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    double step_time, double angular_velocity, std::string rotation_axis,
    bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    tf2::Vector3 relative_translation, res_translation;
    tf2::Quaternion res_quat;
    // translation of new marker in center_point_frame
    visualization_msgs::InteractiveMarkerFeedback relative_curr;
    relative_curr.pose.position.x = curr.pose.position.x - center_point.x;
    relative_curr.pose.position.y = curr.pose.position.y - center_point.y;
    relative_curr.pose.position.z = curr.pose.position.z - center_point.z;
    relative_translation = translationRotate(relative_curr, step_angle, rotation_axis);
    // transform to world frame
    res_translation.setX(relative_translation.x() + center_point.x);
    res_translation.setY(relative_translation.y() + center_point.y);
    res_translation.setZ(relative_translation.z() + center_point.z);

    // if you want to rotate marker orientation together
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = quatRotate(curr, center_point, step_angle, rotation_axis);
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}

// angular position control, with center_frame_quat
void markerPoseAngularPosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const geometry_msgs::Quaternion center_frame_quat,
    const double step_time, const double angular_velocity, const std::string rotation_axis,
    const double delta_angle, bool &is_rotated, bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    // check if marker has reached target angle
    static double accumulated_angle = 0;
    accumulated_angle += step_angle;
    if (accumulated_angle >= delta_angle)
    {
        step_angle = delta_angle - (accumulated_angle - step_angle);
        is_rotated = 0;
        accumulated_angle = 0;
    }
    std::cout << "acc_angle: " << accumulated_angle << std::endl;

    tf2::Vector3 res_translation;
    tf2::Quaternion res_quat;
    tf2::Transform res_tf;
    // rotate (x,y,z) and (qx,qy,qz,qw) together
    res_tf = poseRotate(curr, center_point, center_frame_quat, step_angle, rotation_axis);
    res_translation = res_tf.getOrigin();

    // if you want to rotate marker orientation
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = res_tf.getRotation();
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}

// angular position control, WITHOUT center_frame rotation
void markerPoseAngularPosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const double step_time, const double angular_velocity, const std::string rotation_axis,
    const double delta_angle, bool &is_rotated, bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    // check if marker has reached target angle
    static double accumulated_angle = 0;
    accumulated_angle += step_angle;
    if (accumulated_angle >= delta_angle)
    {
        step_angle = delta_angle - (accumulated_angle - step_angle);
        is_rotated = 0;
        accumulated_angle = 0;
    }
    std::cout << "acc_angle: " << accumulated_angle << std::endl;

    tf2::Vector3 relative_translation, res_translation;
    tf2::Quaternion res_quat;
    // translation of new marker in center_point_frame
    visualization_msgs::InteractiveMarkerFeedback relative_curr;
    relative_curr.pose.position.x = curr.pose.position.x - center_point.x;
    relative_curr.pose.position.y = curr.pose.position.y - center_point.y;
    relative_curr.pose.position.z = curr.pose.position.z - center_point.z;
    relative_translation = translationRotate(relative_curr, step_angle, rotation_axis);
    // transform to world frame
    res_translation.setX(relative_translation.x() + center_point.x);
    res_translation.setY(relative_translation.y() + center_point.y);
    res_translation.setZ(relative_translation.z() + center_point.z);

    // if you want to rotate marker orientation together
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = quatRotate(curr, center_point, step_angle, rotation_axis);
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}