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

#ifndef WORKING_ENVELOPE_VISUALIZER_H
#define WORKING_ENVELOPE_VISUALIZER_H

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf_parser/urdf_parser.h>
#include <eigen_conversions/eigen_msg.h>
#include <bin_pose_msgs/envelope_visualize.h>
#include <bin_pose_msgs/joint_limits.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>

class WorkingEnvelopeVisualizer
{
public:
  WorkingEnvelopeVisualizer(ros::NodeHandle* nh);
  ~WorkingEnvelopeVisualizer();

  bool callback(bin_pose_msgs::envelope_visualize::Request& req, bin_pose_msgs::envelope_visualize::Response& res);
  void visualizePoint(int id, double marker_size,  double x, double y, double z);
  void visualizePose(int id, geometry_msgs::Pose pose);

private:

  std::string robot_description_;
  boost::shared_ptr<urdf::ModelInterface> urdf_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  std::vector<joint_limits_interface::JointLimits> joint_limits_;

  static const int NUM_OF_JOINTS = 6;
  static const int SAMPLING = 20;
  static const int IK_ATTEMPTS = 3;
  static constexpr double IK_TIMEOUT = 0.005;

  ros::ServiceClient joint_limit_client_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher point_pub_;
  int point_id_counter_;
};

#endif // WORKING_ENVELOPE_VISUALIZER_H
