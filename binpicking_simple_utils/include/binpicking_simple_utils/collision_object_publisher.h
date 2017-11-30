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

#ifndef COLLISION_OBJECT_PUBLISHER_H
#define COLLISION_OBJECT_PUBLISHER_H

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

struct CollisionObject
{
  std::string label;
  std::string model_filepath;
  double x_position;
  double y_position;
  double z_position;
  double roll;
  double pitch;
  double yaw;
  double x_scale;
  double y_scale;
  double z_scale;
};

class CollisionObjectPublisher
{
public:
  CollisionObjectPublisher(ros::NodeHandle* nh, std::string co_list_filepath);
  ~CollisionObjectPublisher();
  void publishAllCollisionObjects();
  void publishSingleCollisionObject(CollisionObject single_object);

private:

  std::vector<CollisionObject> collision_objects;
  ros::Publisher pub;
};

#endif // COLLISION_OBJECT_PUBLISHER_H
