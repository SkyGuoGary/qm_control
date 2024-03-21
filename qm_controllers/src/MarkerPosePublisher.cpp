#include <cmath>
#include <geometry_msgs/Twist.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace qm;

const int f = 10;
// position delta between obs & truth
const float delta_x = -0.003;
const float delta_y = -0.0015;
const float delta_z = 0.011;

geometry_msgs::Twist marker_vel;
double pose_x_vel = 0, pose_y_vel = 0, pose_z_vel = 0;
visualization_msgs::InteractiveMarkerFeedback initial_pose;
bool received_curr = 0;

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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "markerPosePub");
    // set initial velocity
    if (argc != 4)
    {
        ROS_ERROR("Usage: markerPosePub <linear_v_x> <linear_v_y> <linear_v_z>");
        return -1;
    }
    pose_x_vel = std::stod(argv[1]);
    pose_y_vel = std::stod(argv[2]);
    pose_z_vel = std::stod(argv[3]);

    // linear_v_z should be 0
    if (fabs(pose_z_vel) >= 0.0001)
    {
        ROS_ERROR("Initial linear_v_z should be 0! ");
        return -2;
    }

    ros::NodeHandle nh;
    ros::Publisher marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    ros::Rate loop_rate(f);

    initial_pose = setInitPose();
    ros::Subscriber marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);
    ros::Subscriber marker_vel_sub = nh.subscribe<geometry_msgs::Twist>("/marker_cmd_vel", 1, markerVelCallback);

    while (!received_curr)
    {
        // waiting for initial ee pose
        ros::spinOnce();
        loop_rate.sleep();
    }
    visualization_msgs::InteractiveMarkerFeedback curr_pose;
    curr_pose = initial_pose;

    while (ros::ok())
    {
        visualization_msgs::InteractiveMarkerFeedback marker_pose;
        pose_x_vel = marker_vel.linear.x;
        pose_y_vel = marker_vel.linear.y;
        pose_z_vel = marker_vel.linear.z;
        marker_pose = markerPoseVelControl(curr_pose, 1.0 / f, pose_x_vel, pose_y_vel, pose_z_vel);
        marker_pose_pub.publish(marker_pose);
        ROS_INFO("marker_x = %lf", marker_pose.pose.position.x);
        ROS_INFO("marker_y = %lf", marker_pose.pose.position.y);
        ROS_INFO("marker_z = %lf", marker_pose.pose.position.z);
        curr_pose = marker_pose;

        ros::spinOnce();
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
    init_pose.pose.orientation.y = 0.707;
    init_pose.pose.orientation.z = 0.0;
    init_pose.pose.orientation.w = 0.707;
    return init_pose;
}