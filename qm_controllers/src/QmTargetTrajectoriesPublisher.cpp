//
// Created by skywoodsz on 2023/3/3.
//

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace qm {
using namespace ocs2;

visualization_msgs::InteractiveMarker QmTargetTrajectoriesInteractiveMarker::createInteractiveMarker() const {
    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = "world";
    interactiveMarker.header.stamp = ros::Time::now();
    interactiveMarker.name = "Goal";
    interactiveMarker.scale = 0.2;
    interactiveMarker.description = "Right click to send command";
    // interactiveMarker.pose.position.x = 0.52;
    // interactiveMarker.pose.position.y = 0.09;
    // interactiveMarker.pose.position.z = 0.436;
    // interactiveMarker.pose.orientation.x = 0.5;
    // interactiveMarker.pose.orientation.y = -0.5;
    // interactiveMarker.pose.orientation.z = 0.5;
    // interactiveMarker.pose.orientation.w = -0.5;

    interactiveMarker.pose.position.x = 0.55;
    interactiveMarker.pose.position.y = 0.175;
    interactiveMarker.pose.position.z = 0.707;
    
    tf2::Quaternion quat_(0, 1/sqrt(2), 0, 1/sqrt(2));
        // tf2::Quaternion quat_(-1/sqrt(2),0, 1/sqrt(2),0);

    quat_.normalize();
    interactiveMarker.pose.orientation.x = quat_.x();
    interactiveMarker.pose.orientation.y = quat_.y();
    interactiveMarker.pose.orientation.z = quat_.z();
    interactiveMarker.pose.orientation.w = quat_.w();
    // create a grey box marker
    const auto boxMarker = []() {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.5;
        return marker;
    }();

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl boxControl;
    boxControl.always_visible = 1;
    boxControl.markers.push_back(boxMarker);
    boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    // add the control to the interactive marker
    interactiveMarker.controls.push_back(boxControl);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    return interactiveMarker;
}

void QmTargetTrajectoriesInteractiveMarker::markerCallback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &msg) {
    const Eigen::Vector3d EePosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const Eigen::Quaterniond EeOrientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z);

    // get TargetTrajectories
    const auto targetTrajectories = goalPoseToTargetTrajectories_(EePosition, EeOrientation,
                                                                  latestObservation_, latestObservationEe_);

    // publish TargetTrajectories
    targetTrajectoriesPublisher_->publishTargetTrajectories(targetTrajectories);

    // update last ee target
    lastEeTarget_ << EePosition, EeOrientation.coeffs();
}

void QmTargetTrajectoriesInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    const Eigen::Vector3d EePosition(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    const Eigen::Quaterniond EeOrientation(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                           feedback->pose.orientation.y, feedback->pose.orientation.z);


    // get TargetTrajectories
    const auto targetTrajectories = goalPoseToTargetTrajectories_(EePosition, EeOrientation,
                                                                  latestObservation_, latestObservationEe_);

    // publish TargetTrajectories
    targetTrajectoriesPublisher_->publishTargetTrajectories(targetTrajectories);

    // update last ee target
    lastEeTarget_ << EePosition, EeOrientation.coeffs();
}

}




