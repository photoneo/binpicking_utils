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
#include <photoneo_msg/operations.h>
#include <photoneo_msg/operation.h>
#include <photoneo_msg/initializePose.h>

// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace NAMES
{
  const static std::string BIN_PICKING = "bin_picking";
  const static std::string BIN_PICKING_SCAN = "scan";
  const static std::string BIN_PICKING_TRAJ = "new_trajectory";
  const static std::string BIN_PICKING_INIT = "initialize_pose";

  const static std::string CALIBRATION_SCAN = "addPoint";
  const static std::string CALIBRATION_COMPUTE = "compute";
  const static std::string CALIBRATION_RESET = "resetCalibration";

  const static std::string AUTO_CALIBRATON = "auto_calibration";

  const static std::string PHO_JOINT_STATES = "photoneo_joint_states";
  const static std::string PHO_TOOL_POSE = "photoneo_tool_pose";

  const static std::string PHOTONEO_DATA = "photoneo_data";
 }

typedef enum REQ
{
  SCAN = 1,
  TRAJECTORY = 2,
  SCAN_AND_TRAJ = 3,
  INITIALIZE = 4,
  CALIBRATION_ADD_POINT_SCAN = 5,
  CALIBRATION_COMPUTE = 6,
  CALIBRATION_RESET = 7,
  AUTO_CALIBRATION = 8,
  NONBLOCK_SCAN = 9,
} request;

typedef enum OPERATION_TYPE
{
  TRAJECTORY_TYPE = 1,
  GRIPPER_TYPE = 2,
  ERROR_TYPE = 3,
  CALIBRATION_TYPE = 4,
  USER_DEFINE_TYPE = 5,
  GRIP_INFO = 6
} operation_type;

typedef enum GRIPPER
{
  GRIPPER_OPEN = 1,
  GRIPPER_CLOSE = 2,
  GRIPPER_AIR_START = 3,
  GRIPPER_AIR_STOP = 4,
  GRIPPER_EJECT = 5,
  GRIPPER_INSERT = 6,
} gripper;

typedef enum BINPICKING_ERROR
{
  SERVICE_ERR = 1,
  UNKNOWN_REQ =2,
  NOT_INITIALIZED = 203,
  NO_PART_FOUND = 202,
  PLANNING_FAILED = 201,
  PART_LOST = 204,
  UNKNOWN_ERROR = 299,
} binpicking_error;

class BinpickingEmulator
{
public:
  BinpickingEmulator(ros::NodeHandle *nh);
  ~BinpickingEmulator();

  bool bin_picking_init_callback(photoneo_msg::initializePose::Request& req, photoneo_msg::initializePose::Response& res);
  bool bin_picking_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res);
  bool bin_picking_traj_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res);
  bool bin_picking_scan_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool calibration_scan_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool calibration_compute_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool auto_calibration_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res);
  bool calibration_reset_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:

  // Variables
  ros::Publisher trajectory_pub_;
  ros::ServiceClient bin_pose_client_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;

  std::vector<double> start_pose_from_robot;
  std::vector<double> end_pose_from_robot;

  int trajectory_marker_index_;

  // Functions
  void visualize_trajectory(trajectory_msgs::JointTrajectory trajectory);

};  //class

#endif  //BINPICKING_EMULATOR_H
