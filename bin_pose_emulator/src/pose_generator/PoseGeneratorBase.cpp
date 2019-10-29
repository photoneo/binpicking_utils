//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

PoseGeneratorBase::PoseGeneratorBase(ros::NodeHandle &nh){
    markerPub = nh.advertise<visualization_msgs::Marker>("bin_pose_visualization", 1);
}

bool PoseGeneratorBase::getPose(geometry_msgs::Pose& pose, double approachDistance) {

    visualizeBin();
    if (!generate(pose)){
        ROS_WARN("Can't generate a new pose");
        return false;
    }

    visualizePose(pose, approachDistance);
    return true;
}

tf::Transform PoseGeneratorBase::broadcastPoseTF(const geometry_msgs::Pose &graspPose) {
    tf::Transform transform;

    tf::poseMsgToTF(graspPose, transform);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "base_link", "current_goal"));
    return transform;
}

void PoseGeneratorBase::visualizePose(const geometry_msgs::Pose& graspPose, double arrowDistance, bool multiArray) {
    broadcastPoseTF(graspPose);

    uint32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "bin";
    if (multiArray){

        static int i = 1;
        marker.id = i++;
    } else {
        marker.id = 1;
    }
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point approachPoint;
    tf::Quaternion quat(graspPose.orientation.x, graspPose.orientation.y,graspPose.orientation.z, graspPose.orientation.w);

    tf::Vector3 vector(0, 0, 1);
    tf::Vector3 rotatedVector = tf::quatRotate(quat, vector);

    approachPoint.x = graspPose.position.x - arrowDistance * rotatedVector.getX();
    approachPoint.y = graspPose.position.y - arrowDistance * rotatedVector.getY();
    approachPoint.z = graspPose.position.z - arrowDistance * rotatedVector.getZ();

    marker.points.push_back(approachPoint);
    marker.points.push_back(graspPose.position);

    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.scale.z = 0.05;

    marker.color.r = 0.9f;
    marker.color.g = 0.9f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    markerPub.publish(marker);
}

sensor_msgs::PointCloud2 PoseGeneratorBase::getPointCloud2() {
    sensor_msgs::PointCloud2 pc2;
    return pc2;
}
