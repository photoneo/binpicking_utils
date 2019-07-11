//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

PoseGeneratorBase::PoseGeneratorBase(ros::NodeHandle &nh){
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("bin_pose_visualization", 1);
}

bool PoseGeneratorBase::getPose(geometry_msgs::Pose &pose){

    visualizeBin();
    generate(pose);
    visualizePose(pose);
}

tf::Transform PoseGeneratorBase::broadcastPoseTF(const geometry_msgs::Pose &grasp_pose)
{
    tf::Transform transform;

    tf::poseMsgToTF(grasp_pose, transform);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "base_link", "current_goal"));
    return transform;
}

void PoseGeneratorBase::visualizePose(geometry_msgs::Pose grasp_pose, double arrow_distance, bool multiArray)
{
    broadcastPoseTF(grasp_pose);

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

    geometry_msgs::Point approach_point;
    tf::Quaternion quat(grasp_pose.orientation.x, grasp_pose.orientation.y,grasp_pose.orientation.z, grasp_pose.orientation.w);

    tf::Vector3 vector(0, 0, 1);
    tf::Vector3 rotated_vector = tf::quatRotate(quat, vector);

    approach_point.x = grasp_pose.position.x - arrow_distance * rotated_vector.getX();
    approach_point.y = grasp_pose.position.y - arrow_distance * rotated_vector.getY();
    approach_point.z = grasp_pose.position.z - arrow_distance * rotated_vector.getZ();

    marker.points.push_back(approach_point);
    marker.points.push_back(grasp_pose.position);

    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.scale.z = 0.05;

    marker.color.r = 0.9f;
    marker.color.g = 0.9f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub_.publish(marker);
}