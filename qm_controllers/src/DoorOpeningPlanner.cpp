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

// fingers[0]~[5]: finger1, finger2, finger3, finger_tip1, finger_tip2, finger_tip3;
ros::Publisher fingers[3];
ros::Publisher grasping_pose_pub, gait_pub;
// grasp action
bool grasped = 0, ungrasped = 1;
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
visualization_msgs::InteractiveMarkerFeedback grasping_point;
// move to grasping point
void move_to_grasp()
{
    grasping_point.pose.position.x = 2.99;
    grasping_point.pose.position.y = 0.175;
    grasping_point.pose.position.z = 1.068;
    tf2::Quaternion grasp_quat_(0,1/sqrt(2),0,1/sqrt(2));
    grasp_quat_.normalize();
    grasping_point.pose.orientation.x = grasp_quat_.x();
    grasping_point.pose.orientation.y = grasp_quat_.y();
    grasping_point.pose.orientation.z = grasp_quat_.z();
    grasping_point.pose.orientation.w = grasp_quat_.w();
    grasping_pose_pub.publish(grasping_point);
    // grasping_pose_pub.publish(target);
}
// rotate handle, angle:degree
void rotate_handle(double angle)
{
    
}
// stage 3: ungrasp handle and push door to a specific angle

// stage 4: move base to go through the door, keep grasped state

// stage 5: ungrasp and reach the final target

int main(int argc, char **argv)
{
    ros::init(argc, argv, "doorOpeningPlanner");
    ros::NodeHandle nh;
    if (argc < 2)
    {
        std::cout << "Usage:" << std::endl
                  << "1:grasp;  0:ungrasp;  2:standing_trot;  3:move to door;  4:stance";
        return 1;
    }
    int stage = std::stoi(argv[1]);
    ros::Rate loop_rate(20);

    grasping_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    gait_pub = nh.advertise<std_msgs::String>("/gait_mode_cmd", 10);
    gait_mode_sub = nh.subscribe<std_msgs::String>("/gait_mode_cmd", 1, gaitCmdCallback);

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

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
