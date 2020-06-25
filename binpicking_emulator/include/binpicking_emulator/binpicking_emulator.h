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
#include <bin_pose_msgs/bin_pose_vector.h>

#include <iostream>
#include <fstream>


// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "binpicking_emulator/kinematic.h"

#define THREADS_COUNT 6
class BinpickingEmulator
{
public:

  typedef std::vector<double> JointValues;

  enum Result {
      OK,
      IK_FAIL,
      PATH_PLAN_FAIL
  };

  BinpickingEmulator(ros::NodeHandle& nh);
  ~BinpickingEmulator();
  void setStartState(const JointValues &start_state);
  void setConstrains();
  Result moveJ(const geometry_msgs::Pose &pose, trajectory_msgs::JointTrajectory &trajectory);
   Result moveJ(const JointValues &joint_pose, trajectory_msgs::JointTrajectory &trajectory);
   std::vector <geometry_msgs::Pose> calculateTrajectoryFK(const trajectory_msgs::JointTrajectory &joint_trajectory);

/*
  void binPickingThreadsLoops();
   void binPickingLoopSimpleTraj();
    void binPickingLoop(int id=0);
    bool makePlan(const geometry_msgs::Pose &pose, bool isLinear = false);
    bool isIKSolutionValid(const planning_scene::PlanningScene* planning_scene,
                           robot_state::RobotState* state,
                           const robot_model::JointModelGroup* jmg,
                           const double* ik_solution);
*/
private:
    static constexpr char* PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_;
    Kinematic kinematic_;


};  // class

#endif  // BINPICKING_EMULATOR_H
