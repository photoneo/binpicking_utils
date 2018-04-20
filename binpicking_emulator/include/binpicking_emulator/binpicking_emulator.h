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
#include <photoneo_msgs/cartesian_goal.h>
#include <photoneo_msgs/initialize_pose.h>
#include <photoneo_msgs/calibration.h>
#include <pho_robot_loader/constants.h>
#include <localization_interface/get_position.h>

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
  bool binPickingTrajArmLeftCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingTrajArmRightCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingCartArmLeftCallback(photoneo_msgs::cartesian_goal::Request& req, photoneo_msgs::cartesian_goal::Response& res);
  bool binPickingCartArmRightCallback(photoneo_msgs::cartesian_goal::Request& req, photoneo_msgs::cartesian_goal::Response& res);
  bool binPickingScanAndTrajCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingInitCallback(photoneo_msgs::initialize_pose::Request& req, photoneo_msgs::initialize_pose::Response& res);
  bool binPickingInitArmLeftCallback(photoneo_msgs::initialize_pose::Request& req, photoneo_msgs::initialize_pose::Response& res);
  bool binPickingInitArmRightCallback(photoneo_msgs::initialize_pose::Request& req, photoneo_msgs::initialize_pose::Response& res);
  bool calibrationAddPointCallback(photoneo_msgs::calibration::Request& req, photoneo_msgs::calibration::Response& res);
  bool calibrationSetToScannerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool calibrationResetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
  // Variables
  ros::ServiceClient bin_pose_client_;
  ros::ServiceClient bin_pose_arm_left_client_;
  ros::ServiceClient bin_pose_arm_right_client_;
  ros::ServiceClient localization_arm_left_client_;
  ros::ServiceClient localization_arm_right_client_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::planning_interface::MoveGroupInterfacePtr group_manipulator_;
  moveit::planning_interface::MoveGroupInterfacePtr group_arm_left_;
  moveit::planning_interface::MoveGroupInterfacePtr group_arm_right_;

  int num_of_joints_;
  std::vector<double> start_pose_from_robot_;
  std::vector<double> end_pose_from_robot_;
  std::vector<double> arm_left_start_pose_from_robot_;
  std::vector<double> arm_left_end_pose_from_robot_;
  std::vector<double> arm_right_start_pose_from_robot_;
  std::vector<double> arm_right_end_pose_from_robot_;

  int trajectory_marker_index_;

};  // class

#endif  // BINPICKING_EMULATOR_H
