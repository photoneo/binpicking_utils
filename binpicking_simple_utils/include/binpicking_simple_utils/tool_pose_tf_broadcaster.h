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

#ifndef TOOL_POSE_TF_BROADCASTER_H
#define TOOL_POSE_TF_BROADCASTER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

class Broadcaster
{
public:
  Broadcaster();
  ~Broadcaster();
  void poseCallback(const geometry_msgs::PoseConstPtr& msg);

private:
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Vector3 origin;
  tf::Quaternion orientation;  
};

#endif // TOOL_POSE_TF_BROADCASTER_H