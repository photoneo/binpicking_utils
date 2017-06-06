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

#ifndef BIN_POSE_EMULATOR_H
#define BIN_POSE_EMULATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <bin_pose_msgs/bin_pose.h>

struct ConfigData
{
  // Virtual Bin center
  double bin_center_x;
  double bin_center_y;
  double bin_center_z;

  // Virtual Bin size
  double bin_size_x;
  double bin_size_y;
  double bin_size_z;

  // Default tool point orientation
  double roll_default;
  double pitch_default;
  double yaw_default;

  // Allowed orientation range
  double roll_range;
  double pitch_range;
  double yaw_range;

  // Planning constraints
  double approach_distance;
  double deapproach_height;
};

class Emulator
{
public:
  Emulator(ros::NodeHandle* nh, std::string filepath);
  ~Emulator();

  bool callback(bin_pose_msgs::bin_pose::Request& req,
                bin_pose_msgs::bin_pose::Response& res);

private:
  double randGen(double fMin, double fMax);
  bool parseConfig(std::string filepath);

  void visualize_bin(void);
  void visualize_pose(geometry_msgs::Pose grasp_pose,
                      geometry_msgs::Pose approach_pose);
  void broadcast_pose_tf(geometry_msgs::Pose grasp_pose);

  ros::Publisher marker_pub;

  ConfigData config;
};

#endif // BIN_POSE_EMULATOR_H
