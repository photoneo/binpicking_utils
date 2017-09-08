/*********************************************************************
Copyright [2017] [Frantisek Durovsky]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 *********************************************************************/

#include <binpicking_simple_utils/tool_pose_tf_broadcaster.h>

Broadcaster::Broadcaster()
{
  
}

Broadcaster::~Broadcaster()
{
  
}

void Broadcaster::poseCallback(const geometry_msgs::PoseConstPtr& msg){
  
  // Check data validity (Quaternion x^2 + y^2 + z^2 + w^2 = 1)
  double quaternion_sum = pow(msg->orientation.x, 2)
                        + pow(msg->orientation.y, 2) 
                        + pow(msg->orientation.z, 2) 
                        + pow(msg->orientation.w, 2);
  
  if (abs(1 -quaternion_sum) < 0.01)
  {
    // Tool Pose transform
    origin.setX(msg->position.x);
    origin.setY(msg->position.y);
    origin.setZ(msg->position.z);
    
    orientation.setX(msg->orientation.x);
    orientation.setY(msg->orientation.y);
    orientation.setZ(msg->orientation.z);
    orientation.setW(msg->orientation.w);
        
    transform.setOrigin(origin);    
    transform.setRotation(orientation);
        
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "real_robot_tool_pose"));
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tool_pose_tf_broadcaster");
  ros::NodeHandle nh;
  
  Broadcaster broadcaster;
  
  ros::Subscriber sub = nh.subscribe("real_robot_tool_pose", 1, &Broadcaster::poseCallback, &broadcaster);

  ROS_INFO("Tool Pose TF Broadcater running!");
  ros::spin();
  return 0;
}