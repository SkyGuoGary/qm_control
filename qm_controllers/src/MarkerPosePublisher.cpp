#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace qm;

const int f = 10;
double pose_x_vel = 0.2, pose_y_vel = 0, pose_z_vel = 0;
visualization_msgs::InteractiveMarkerFeedback initial_pose;
bool received_curr=0;
visualization_msgs::InteractiveMarkerFeedback setInitPose();
visualization_msgs::InteractiveMarkerFeedback markerPoseVelControl(
    visualization_msgs::InteractiveMarkerFeedback curr,
    double step_time, double v_x, double v_y, double v_z)
{
    curr.pose.position.x += v_x * step_time;
    curr.pose.position.y += v_y * step_time;
    curr.pose.position.z += v_z * step_time;
    return curr;
}

void feedbackCallback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &msg)
{
    // 获取接收到的消息中的位置信息
    initial_pose.pose = msg->pose;
    received_curr=1;
    ROS_INFO("CURRENT POSE: %lf, %lf, %lf",
             initial_pose.pose.position.x, initial_pose.pose.position.y,
             initial_pose.pose.position.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "markerPosePub");
    ros::NodeHandle nh;
    ros::Publisher marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    ros::Rate loop_rate(f);

    initial_pose = setInitPose();
    ros::Subscriber marker_feedback_sub = nh.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/simple_marker/feedback", 1, feedbackCallback);
    while (!received_curr)
    {
        // 等待初始位置的订阅消息
        ros::spinOnce();
        loop_rate.sleep();
    }
    visualization_msgs::InteractiveMarkerFeedback curr_pose;
    curr_pose = initial_pose;

    while (ros::ok())
    {
        visualization_msgs::InteractiveMarkerFeedback marker_pose;
        marker_pose = markerPoseVelControl(curr_pose, 1.0 / f, pose_x_vel, pose_y_vel, pose_z_vel);
        marker_pose_pub.publish(marker_pose);
        ROS_INFO("publish x=%lf", marker_pose.pose.position.x);
        curr_pose = marker_pose;
        // 循环等待回调函数
        ros::spinOnce();
        // 按照循环频率延时
        loop_rate.sleep();
    }
    return 0;
}

visualization_msgs::InteractiveMarkerFeedback setInitPose()
{
    visualization_msgs::InteractiveMarkerFeedback init_pose;
    init_pose.pose.position.x = 0.55;
    init_pose.pose.position.y = 0.175;
    init_pose.pose.position.z = 0.7;
    init_pose.pose.orientation.x = 0.0;
    init_pose.pose.orientation.y = -0.707;
    init_pose.pose.orientation.z = 0.0;
    init_pose.pose.orientation.w = -0.707;
    return init_pose;
}