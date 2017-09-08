#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

static tf::TransformBroadcaster br;

void poseCallback(const geometry_msgs::PoseConstPtr& msg){
  
  // Check data validity (Quaternion x^2 + y^2 + z^2 + w^2 = 1)
  double quaternion_sum = pow(msg->orientation.x, 2)
                        + pow(msg->orientation.y, 2) 
                        + pow(msg->orientation.z, 2) 
                        + pow(msg->orientation.w, 2);
  
  if (abs(1 -quaternion_sum) < 0.01)
  {
    // Tool Pose transform
    tf::Vector3 origin(msg->position.x, msg->position.y, msg->position.z);
    tf::Quaternion orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    
    tf::Transform transform;
    transform.setOrigin(origin);    
    transform.setRotation(orientation);
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "real_robot_tool_pose"));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tool_pose_tf_broadcaster");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("real_robot_tool_pose", 1, &poseCallback);

  ROS_INFO("Tool Pose TF Broadcater running!");
  ros::spin();
  return 0;
}