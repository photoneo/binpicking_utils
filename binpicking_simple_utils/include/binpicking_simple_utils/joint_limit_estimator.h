/*********************************************************************
Copyright [2018] [Frantisek Durovsky]

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

#ifndef JOINT_LIMIT_ESTIMATOR_H
#define JOINT_LIMIT_ESTIMATOR_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <urdf_model/model.h>
#include <bin_pose_msgs/joint_limits.h>
#include <eigen_conversions/eigen_msg.h>
#include <urdf_parser/urdf_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class JointLimitEstimator
{
public:
  JointLimitEstimator(ros::NodeHandle* nh);
  ~JointLimitEstimator();

  bool callback(bin_pose_msgs::joint_limits::Request& req, bin_pose_msgs::joint_limits::Response& res);
  double estimateJointLimits(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2, int iterations);
  Eigen::Affine3d generateRandomBinPose(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2);
  void visualizeBin(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2);
  double randGen(double fMin, double fMax);

private:

  std::string robot_description_;
  boost::shared_ptr<urdf::ModelInterface> urdf_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  std::vector<joint_limits_interface::JointLimits> joint_limits_;
  ros::Publisher marker_pub_;

  static const int NUM_OF_JOINTS = 6;
  static const int NUM_OF_INTERVALS = 127;
  static const int IK_ATTEMPTS = 3;
  static constexpr double IK_TIMEOUT = 0.005;

  static constexpr double ALLOWED_ROLL_RANGE = 0.707;   // rad
  static constexpr double ALLOWED_PITCH_RANGE = 0.707;  // rad
  static constexpr double ALLOWED_YAW_RANGE = 0.707;    // rad

  static constexpr double ACCEPTANCE_MIN_THRESHOLD = 0.05;
  static constexpr double ACCEPTANCE_MAX_THRESHOLD = 0.95;

};

#endif // JOINT_LIMIT_ESTIMATOR_H
