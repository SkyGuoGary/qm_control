#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/CommandLine.h>

#include <qm_controllers/GaitSwitch.h>
#include <qm_controllers/MarkerPosePublisher.h>

using namespace ocs2;
using namespace qm;
using namespace legged_robot;
// ros param
double freq = 10;
double step_time = 1.0 / freq;
// fingers[0]~[5]: finger1, finger2, finger3, finger_tip1, finger_tip2, finger_tip3;
ros::Publisher fingers[3];
ros::Publisher gait_pub;
// grasp action
bool grasped = 0, ungrasped = 1;
bool reached = 0, is_rotated = 1;

void grasp_door_handle(bool ungrasp = 0)
{
    std_msgs::Float64 fingers_val[3];
    // ungrasp
    if (ungrasp)
    {
        for (int i = 0; i < 3; i++)
        {
            fingers_val[i].data = 0;
            if (!ungrasped)
            {
                std::cout << "Command: ungrasp handle" << std::endl;
                ungrasped = 1;
                grasped = 0;
            }
            fingers[i].publish(fingers_val[i]);
        }
        return;
    }

    // grasp
    fingers_val[0].data = 1.2;
    fingers_val[1].data = 1.22;
    fingers_val[2].data = 0;
    if (!grasped)
    {
        std::cout << "Command: start grasping" << std::endl;
        printf("finger 1~3: %lf,%lf,%lf",
               fingers_val[0].data, fingers_val[1].data, fingers_val[2].data);
        grasped = 1;
        ungrasped = 0;
    }
    for (int i = 0; i < 3; i++)
    {
        fingers[i].publish(fingers_val[i]);
    }
}
// switch gait mode
void switch_gait_pub(myGaitKeyboardPublisher myGaitCommand, std::string gaitName)
{
    std_msgs::String init_gait;
    init_gait.data = gaitName;
    gait_pub.publish(init_gait);
    if (is_converted)
    {
        myGaitCommand.switchGaitMode(sub_gait_mode);
    }
}
// subcribe grasping point
geometry_msgs::Transform set_grasping_pose()
{
    tf2::Vector3 grasp_point_(2.99, 0.175, 1.068);
    geometry_msgs::Vector3 grasp_point = tf2::toMsg(grasp_point_);

    tf2::Quaternion grasp_quat_(0, 1 / sqrt(2), 0, 1 / sqrt(2)); // rotate 90 degree around world_Y
    grasp_quat_ = grasp_quat_.normalize();
    geometry_msgs::Quaternion grasp_quat = tf2::toMsg(grasp_quat_);

    geometry_msgs::Transform grasping_pose;
    grasping_pose.translation = grasp_point;
    grasping_pose.rotation = grasp_quat;
    return grasping_pose;
}
// move to grasping point
void move_to_grasp(double v_max = 0.15)
{
    geometry_msgs::Transform target_pose = set_grasping_pose();

    geometry_msgs::Vector3 target_position;
    target_position.x = target_pose.translation.x;
    target_position.y = target_pose.translation.y;
    target_position.z = target_pose.translation.z;

    marker_pose.pose.orientation = target_pose.rotation;
    if (!reached)
    {
        markerPosePosControl(marker_pose, target_position, step_time, v_max, reached, 1);
    }
}
// subscribe handle frame for rotation
geometry_msgs::Transform set_handle_frame()
{
    tf2::Vector3 handle_point_(2.99, 0.225, 1.068);
    geometry_msgs::Vector3 handle_point = tf2::toMsg(handle_point_);

    tf2::Quaternion handle_quat_(0, 1 / sqrt(2), 0, 1 / sqrt(2)); // rotate 90 degree around world_Y
    handle_quat_ = handle_quat_.normalize();
    geometry_msgs::Quaternion handle_quat = tf2::toMsg(handle_quat_);

    geometry_msgs::Transform handle_frame;
    handle_frame.translation = handle_point;
    handle_frame.rotation = handle_quat;
    return handle_frame;
}
// rotate handle, angle:radian, omega:rad/s
void rotate_handle(double angle, double omega=0.2)
{
    geometry_msgs::Transform handle_frame=set_handle_frame();
    if (is_rotated)
    {
        markerPoseAngularPosControl(
            marker_pose, handle_frame.translation, handle_frame.rotation,
            step_time, omega, "z", angle, is_rotated);
    }
}
// stage 3: ungrasp handle and push door to a specific angle

// stage 4: move base to go through the door, keep grasped state

// stage 5: ungrasp and reach the final target

int main(int argc, char **argv)
{
    ros::init(argc, argv, "doorOpeningPlanner");
    if (argc < 2)
    {
        std::cout << "Usage:" << std::endl
                  << "1:grasp;  0:ungrasp;  2:standing_trot;  3:move to door;  4:stance";
        return 1;
    }
    int stage = std::stoi(argv[1]);

    ros::NodeHandle nh;
    marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    gait_pub = nh.advertise<std_msgs::String>("/gait_mode_cmd", 10);
    gait_mode_sub = nh.subscribe<std_msgs::String>("/gait_mode_cmd", 1, gaitCmdCallback);
    marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);

    ros::Rate loop_rate(freq);

    initial_pose = setInitPose(); // only initailized, not real initail pose
    // waiting for initial ee pose
    while (!received_curr_marker && ros::ok() && ros::master::check())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    marker_pose = initial_pose;

    // Get gait command file
    nh.getParam("/gaitCommandFile", gaitCommandFile);
    myGaitKeyboardPublisher gaitCommand(nh, gaitCommandFile, "legged_robot", true);

    // Enable finger position publishers
    for (int i = 0; i < 3; i++)
    {
        fingers[i] = nh.advertise<std_msgs::Float64>("/finger_" + std::to_string(i + 1) + "_position_controller/command", 50);
        // fingers[i + 3] = nh.advertise<std_msgs::Float64>("/finger_tip_" + std::to_string(i + 1) + "_position_controller/command", 50);
    }
    bool standing_trot = 0;
    while (ros::ok())
    {
        // stage 1: move to grasping point
        // stage 2: stop base, grasp handle and rotate
        // stage 3: ungrasp handle and push door to a specific angle
        // stage 4: move base to go through the door
        // stage 5: ungrasp and reach the final target
        if (stage == 1 || stage == 0)
        {
            grasp_door_handle(!stage);
        }
        if (stage == 2)
        {
            switch_gait_pub(gaitCommand, "standing_trot");
        }
        if (stage == 3)
        {
            move_to_grasp();
        }
        if (stage == 4)
        {
            switch_gait_pub(gaitCommand, "stance");
        }
        if (stage == 5)
        {
            double rotate_angle = 45 * M_PI / 180.0;
            rotate_handle(rotate_angle);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
