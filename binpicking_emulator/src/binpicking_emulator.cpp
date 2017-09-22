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

#include "binpicking_emulator/binpicking_emulator.h"


BinpickingEmulator::BinpickingEmulator(ros::NodeHandle* nh) :
  trajectory_marker_index_(0)
{
  // Initialize Moveit group
  group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

  // Load robot description
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  // Configure bin pose client
  bin_pose_client_ = nh->serviceClient<bin_pose_msgs::bin_pose>("bin_pose");

  // Set trajectory visualization publisher
  trajectory_pub_ = nh->advertise<visualization_msgs::Marker>("trajectory", 1);

  // Set move group params
  group_->setPlannerId("RRTConnectkConfigDefault");
  group_->setGoalTolerance(0.001);
}

BinpickingEmulator::~BinpickingEmulator()
{

}

bool BinpickingEmulator::bin_picking_init_callback(photoneo_msg::initializePose::Request& req, photoneo_msg::initializePose::Response& res)
{
  ROS_INFO("Binpicking Emulator: Binpicking Init Service called");

  ROS_INFO("START POSE: [%f, %f, %f, %f, %f, %f]",
      req.startPose.position[0],
      req.startPose.position[1],
      req.startPose.position[2],
      req.startPose.position[3],
      req.startPose.position[4],
      req.startPose.position[5]);

  ROS_INFO("END POSE: [%f, %f, %f, %f, %f, %f]",
      req.endPose.position[0],
      req.endPose.position[1],
      req.endPose.position[2],
      req.endPose.position[3],
      req.endPose.position[4],
      req.endPose.position[5]);

  res.success = true;
  res.result = 0;
  return true;
}

bool BinpickingEmulator::bin_picking_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res)
{
  ROS_INFO("Binpicking Emulator: Binpicking Service called");
  return true;
}

bool BinpickingEmulator::bin_picking_traj_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res)
{
  // Get current state
  robot_state::RobotState current_state(*group_->getCurrentState());

  //---------------------------------------------------
  // Set Start state
  //---------------------------------------------------
  int start_traj_size;
  moveit::planning_interface::MoveGroupInterface::Plan to_start_pose;
  group_->setNamedTarget("start_pose");
  group_->plan(to_start_pose);
  start_traj_size = to_start_pose.trajectory_.joint_trajectory.points.size();
  current_state.setJointGroupPositions("manipulator", to_start_pose.trajectory_.joint_trajectory.points[start_traj_size-1].positions);
  group_->setStartState(current_state);

  // Get random bin picking pose from emulator
  bin_pose_msgs::bin_pose srv;  
  geometry_msgs::Pose approach_pose, grasp_pose, deapproach_pose;

  if(bin_pose_client_.call(srv))
  {
    grasp_pose = srv.response.grasp_pose;
    approach_pose = srv.response.approach_pose;
    deapproach_pose = srv.response.deapproach_pose;
  }

  //---------------------------------------------------
  // Plan trajectory from current to approach pose
  //---------------------------------------------------
  int approach_traj_size;
  moveit::planning_interface::MoveGroupInterface::Plan to_approach_pose;
  group_->setPoseTarget(approach_pose);
  bool success_approach = group_->plan(to_approach_pose);
  if(success_approach)
  {
    // Get trajectory size from plan
    approach_traj_size = to_approach_pose.trajectory_.joint_trajectory.points.size();

    // SetStartState instead of trajectory execution
    current_state.setJointGroupPositions("manipulator", to_approach_pose.trajectory_.joint_trajectory.points[approach_traj_size-1].positions);
    group_->setStartState(current_state);

    // Visualize trajectory in RViz
    visualize_trajectory(to_approach_pose.trajectory_.joint_trajectory);
  }

  //---------------------------------------------------
  // Plan trajectory from approach to grasp pose
  //---------------------------------------------------
  int grasp_traj_size;
  moveit::planning_interface::MoveGroupInterface::Plan to_grasp_pose;
  group_->setPoseTarget(grasp_pose);
  bool success_grasp = group_->plan(to_grasp_pose);
  if(success_grasp)
  {
    // Get trajectory size from plan
    grasp_traj_size = to_grasp_pose.trajectory_.joint_trajectory.points.size();

    // SetStartState instead of trajectory execution
    current_state.setJointGroupPositions("manipulator", to_grasp_pose.trajectory_.joint_trajectory.points[grasp_traj_size-1].positions);
    group_->setStartState(current_state);

    // Visualize trajectory in RViz
    visualize_trajectory(to_grasp_pose.trajectory_.joint_trajectory);
  }

  //---------------------------------------------------
  // Plan trajectory from grasp to deapproach pose
  //---------------------------------------------------
  int deapproach_traj_size;
  moveit::planning_interface::MoveGroupInterface::Plan to_deapproach_pose;
  group_->setPoseTarget(deapproach_pose);
  bool success_deapproach = group_->plan(to_deapproach_pose);
  if(success_deapproach)
  {
    // Get trajectory size from plan
    deapproach_traj_size = to_deapproach_pose.trajectory_.joint_trajectory.points.size();

    // SetStartState instead of trajectory execution
    current_state.setJointGroupPositions("manipulator", to_deapproach_pose.trajectory_.joint_trajectory.points[deapproach_traj_size-1].positions);
    group_->setStartState(current_state);

    // Visualize trajectory in RViz
    visualize_trajectory(to_deapproach_pose.trajectory_.joint_trajectory);
  }

  //---------------------------------------------------
  // Plan trajectory from deapproach to end pose
  //---------------------------------------------------
  int end_traj_size;
  moveit::planning_interface::MoveGroupInterface::Plan to_end_pose;
  group_->setNamedTarget("end_pose");
  bool success_end = group_->plan(to_end_pose);
  if(success_end)
  {
    // Get trajectory size from plan
    end_traj_size = to_end_pose.trajectory_.joint_trajectory.points.size();

    // SetStartState instead of trajectory execution
    current_state.setJointGroupPositions("manipulator", to_end_pose.trajectory_.joint_trajectory.points[end_traj_size-1].positions);
    group_->setStartState(current_state);

    // Visualize trajectory in RViz
    visualize_trajectory(to_end_pose.trajectory_.joint_trajectory);
  }

  //---------------------------------------------------
  // Compose binpicking as a sequence of operations
  //---------------------------------------------------

  if((success_approach) && (success_grasp) && (success_deapproach) && (success_end))
  {
    photoneo_msg::operation binpicking_operation;

    // Operation 1 - Approach Trajectory
    binpicking_operation.msgType = OPERATION_TYPE::TRAJECTORY_TYPE;

    binpicking_operation.points.clear();
    for(int i = 0; i < approach_traj_size; i++)
      binpicking_operation.points.push_back(to_approach_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = GRIPPER::GRIPPER_OPEN;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 2 - Open Gripper
    binpicking_operation.msgType = OPERATION_TYPE::GRIPPER_TYPE;
    binpicking_operation.points.clear();
    binpicking_operation.gripper = GRIPPER::GRIPPER_OPEN;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 3 - Grasp Trajectory
    binpicking_operation.msgType = OPERATION_TYPE::TRAJECTORY_TYPE;

    binpicking_operation.points.clear();
    for(int i = 0; i < grasp_traj_size; i++)
      binpicking_operation.points.push_back(to_grasp_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = GRIPPER::GRIPPER_OPEN;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 4 - Close Gripper
    binpicking_operation.msgType = OPERATION_TYPE::GRIPPER_TYPE;
    binpicking_operation.points.clear();
    binpicking_operation.gripper = GRIPPER::GRIPPER_CLOSE;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 5 - Deapproach trajectory
    binpicking_operation.msgType = OPERATION_TYPE::TRAJECTORY_TYPE;

    binpicking_operation.points.clear();
    for(int i = 0; i < deapproach_traj_size; i++)
      binpicking_operation.points.push_back(to_deapproach_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = GRIPPER::GRIPPER_CLOSE;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 6 - End Trajectory
    binpicking_operation.msgType = OPERATION_TYPE::TRAJECTORY_TYPE;

    binpicking_operation.points.clear();
    for(int i = 0; i < end_traj_size; i++)
      binpicking_operation.points.push_back(to_end_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = GRIPPER::GRIPPER_CLOSE;
    binpicking_operation.calibration = 0;
    binpicking_operation.userDefine = 0;
    binpicking_operation.errorCode = 0;
    binpicking_operation.gripInfo = 0;

    res.operations.push_back(binpicking_operation);

    return true;
  }
  else
  {
    ROS_WARN("Planning failed, repeat the request");
    return false;
  }
}

bool BinpickingEmulator::bin_picking_scan_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Binpicking Emulator: Binpicking Scan Service called");
  res.success = true;
  return true;
}

bool BinpickingEmulator::calibration_scan_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Binpicking Emulator: Calibration Scan Service called");
  res.success = true;
  return true;
}

bool BinpickingEmulator::calibration_compute_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Binpicking Emulator: Calibration Compute Service called");
  res.success = true;
  return true;
}

bool BinpickingEmulator::calibration_reset_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Binpicking Emulator: Calibration Compute Service called");
  res.success = true;
  return true;
}


bool BinpickingEmulator::auto_calibration_callback(photoneo_msg::operations::Request& req, photoneo_msg::operations::Response& res)
{
  ROS_INFO("Binpicking Emulator: Auto Calibration Service called");
  return true;
}

void BinpickingEmulator::visualize_trajectory(trajectory_msgs::JointTrajectory trajectory)
{
  visualization_msgs::Marker marker;

  // Kinematic variables
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  marker.header.frame_id = "/base_link";
  marker.ns = "trajectory";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  for(int i = 0; i < trajectory.points.size(); i++)
  {
    kinematic_state->setJointGroupPositions("manipulator", trajectory.points[i].positions);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    marker.header.stamp = ros::Time::now();
    marker.id = trajectory_marker_index_++;

    marker.pose.position.x = end_effector_state.translation()[0];
    marker.pose.position.y = end_effector_state.translation()[1];
    marker.pose.position.z = end_effector_state.translation()[2];

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.9f;
    marker.color.g = 0.9f;
    marker.color.b = 0.9f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);
    trajectory_pub_.publish(marker);
    ros::Duration(0.001).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "binpicking_emulator");
  ros::NodeHandle nh;

  // Wait until moveit config is properly loaded
  ros::Duration(10).sleep();

  // Create BinpickingEmulator instance
  BinpickingEmulator emulator(&nh);

  // Advertise service
  ros::ServiceServer bin_picking_service = nh.advertiseService(NAMES::BIN_PICKING, &BinpickingEmulator::bin_picking_callback, &emulator);
  ros::ServiceServer bin_picking_scan_service = nh.advertiseService(NAMES::BIN_PICKING_SCAN, &BinpickingEmulator::bin_picking_scan_callback, &emulator);
  ros::ServiceServer bin_picking_traj_service = nh.advertiseService(NAMES::BIN_PICKING_TRAJ, &BinpickingEmulator::bin_picking_traj_callback, &emulator);
  ros::ServiceServer bin_picking_init_service = nh.advertiseService(NAMES::BIN_PICKING_INIT, &BinpickingEmulator::bin_picking_init_callback, &emulator);

  ros::ServiceServer calibration_scan_service = nh.advertiseService(NAMES::CALIBRATION_SCAN, &BinpickingEmulator::calibration_scan_callback, &emulator);
  ros::ServiceServer calibration_compute_service = nh.advertiseService(NAMES::CALIBRATION_COMPUTE, &BinpickingEmulator::calibration_compute_callback, &emulator);
  ros::ServiceServer calibration_reset_service = nh.advertiseService(NAMES::CALIBRATION_RESET, &BinpickingEmulator::calibration_reset_callback, &emulator);
  ros::ServiceServer auto_calibration_service = nh.advertiseService(NAMES::AUTO_CALIBRATON, &BinpickingEmulator::auto_calibration_callback, &emulator);

  ROS_INFO("Binpicking Emulator Ready");

  // Start Async Spinner with 2 threads
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}
