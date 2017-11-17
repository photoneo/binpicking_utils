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

#ifndef BINPICKING_EMULATOR_H
#define BINPICKING_EMULATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <bin_pose_msgs/bin_pose.h>
#include <photoneo_msgs/operations.h>
#include <photoneo_msgs/operation.h>
#include <photoneo_msgs/initialize_pose.h>
#include <photoneo_msgs/calibration.h>
#include <pho_robot_loader/constants.h>

// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class BinpickingEmulator
{
public:
  BinpickingEmulator(ros::NodeHandle* nh);
  ~BinpickingEmulator();

  bool binPickingScanCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool binPickingTrajCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingScanAndTrajCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingInitCallback(photoneo_msgs::initialize_pose::Request& req, photoneo_msgs::initialize_pose::Response& res);
  bool calibrationAddPointCallback(photoneo_msgs::calibration::Request& req, photoneo_msgs::calibration::Response& res);
  bool calibrationSetToScannerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool calibrationResetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
  // Variables
  ros::Publisher trajectory_pub_;
  ros::ServiceClient bin_pose_client_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;

  std::vector<double> start_pose_from_robot_;
  std::vector<double> end_pose_from_robot_;

  int trajectory_marker_index_;

  // Functions
  void visualizeTrajectory(trajectory_msgs::JointTrajectory trajectory);

};  // class

#endif  // BINPICKING_EMULATOR_H
