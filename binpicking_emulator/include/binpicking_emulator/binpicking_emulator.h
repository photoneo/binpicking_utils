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
#include <photoneo_msgs/add_point.h>
#include <pho_robot_loader/constants.h>

#include <iostream>
#include <fstream>


// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <stomp_param_changer/statistics.h>

#include "binpicking_emulator/path_length_test.h"


class BinpickingEmulator
{
public:
    struct Waypoint{
        bool is_linear;
        bool is_joint_space;
        geometry_msgs::Pose pose;
        std::vector<double> end_joint_state;
    };

  BinpickingEmulator(ros::NodeHandle* nh);
  ~BinpickingEmulator();

  bool binPickingScanCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool binPickingTrajCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingScanAndTrajCallback(photoneo_msgs::operations::Request& req, photoneo_msgs::operations::Response& res);
  bool binPickingInitCallback(photoneo_msgs::initialize_pose::Request& req, photoneo_msgs::initialize_pose::Response& res);
  bool calibrationAddPointCallback(photoneo_msgs::add_point::Request& req, photoneo_msgs::add_point::Response& res);
  bool calibrationSetToScannerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool calibrationResetCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void binPickingLoopSimpleTraj();
    void binPickingLoop();
    bool makePlan(const geometry_msgs::Pose &pose, bool isLinear = false);
    bool isIKSolutionValid(const planning_scene::PlanningScene* planning_scene,
                           robot_state::RobotState* state,
                           const robot_model::JointModelGroup* jmg,
                           const double* ik_solution);

private:
  // Variables
  ros::Publisher trajectory_pub_;
  ros::ServiceClient bin_pose_client_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;


    void publishResult();
    void writeToFile();
    void createStatistics(moveit::planning_interface::MoveItErrorCode success, trajectory_msgs::JointTrajectory trajectory, double time, const geometry_msgs::Pose pose);

    bool checkCartesianContinuity(moveit_msgs::RobotTrajectory &trajectory, float limit);

    int num_of_joints_;
  std::vector<double> start_pose_from_robot_;
  std::vector<double> end_pose_from_robot_;

  int trajectory_marker_index_;

  // Functions
  void visualizeTrajectory(trajectory_msgs::JointTrajectory trajectory);

    std::ofstream outfile_fails_stomp_;
    std::ofstream outfile_fails_ik_;
    std::ofstream outfile_joint_diff_;
    std::ofstream outfile_points_;


    ros::Publisher statistics_pub_;

    double average_time_;
    int num_of_attempt_;
    int num_of_success_;
    int num_of_fails_;
    double sum_time_;
    double sum_joint_diff_;
    double sum_traj_size_;
    double average_joint_diff_;
    double average_traj_size_;
    int ik_fails_sum_;
    double success_rate_;
    int bad_trajectory_;

    std::string log_path_;
    PathLengthTest path_length_test_;

    std::vector<int> ik_fails_;
    std::vector<int> planner_fails_;
    std::vector<int> continuity_checker_;
    int point_id_;
    geometry_msgs::Point last_point_;

    //STOMP statistics
    std::ofstream outfile_stomp_stats_;
    double stompTempTime;
    int stompSuccess; // 0-STOMP failed, 1-Success, 2-IK failed or linear planner failed
    double stompSumTime;
    int stompNumOfSuccess;
    int stompNumOfFails;
    int stompNumOfPicks;
    double stompPathLength;
    double stompSumLength;
    void createStompStatistics(int success, double time, double pathLength);

};  // class

#endif  // BINPICKING_EMULATOR_H
