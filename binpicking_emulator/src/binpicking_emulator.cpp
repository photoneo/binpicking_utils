/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <bin_pose_msgs/bin_pose_vector.h>
#include <binpicking_emulator/binpicking_emulator.h>

#include <eigen_conversions/eigen_msg.h>

BinpickingEmulator::BinpickingEmulator(ros::NodeHandle& nh) :
    move_group_(PLANNING_GROUP),
    //robotModelLoader("robot_description"),
    kinematic_(robot_model_loader::RobotModelLoader("robot_description").getModel()),
    evaluator()
    {

    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
   // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
 //   const robot_state::JointModelGroup* joint_model_group =
//            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_.getJointModelGroupNames().begin(), move_group_.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

}

BinpickingEmulator::~BinpickingEmulator(){
}

void BinpickingEmulator::setStartState(const JointValues &start_state) {
    robot_state::RobotStatePtr current_state = move_group_.getCurrentState();
    current_state->setJointGroupPositions("manipulator", start_state);
    move_group_.setStartState(*current_state);
}

bool BinpickingEmulator::moveJ(const JointValues &joint_pose) {
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_.setJointValueTarget(joint_pose);
    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    inputTrajectory trajectory;
    trajectory.jointPositions = my_plan.trajectory_.joint_trajectory;
    trajectory.pose = calculateTrajectoryFK(trajectory.jointPositions);
    return success;
}

bool BinpickingEmulator::moveJ(const geometry_msgs::Pose &pose) {
  //  move_group_.setPoseTarget(pose);
    JointValues joint_values;
    kinematic_.getIK(pose, joint_values);
   return moveJ(joint_values);
}

std::vector <geometry_msgs::Pose> BinpickingEmulator::calculateTrajectoryFK(const trajectory_msgs::JointTrajectory &joint_trajectory) {
    std::vector <geometry_msgs::Pose> poses;

    for (auto &point : joint_trajectory.points) {
        const Eigen::Affine3d &end_effector_state = kinematic_.getFK(point.positions);
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);
        poses.push_back(pose);
    }
    return poses;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  BinpickingEmulator planner(node_handle);

  const BinpickingEmulator::JointValues start = {0, 0, 0, 0, 0, 0};
  const BinpickingEmulator::JointValues end = {1.5, 0, 0, 0, 0, 0};
  planner.setStartState(start);

    // Configure bin pose client
    ros::ServiceClient bin_pose_client_ = node_handle.serviceClient<bin_pose_msgs::bin_pose_vector>("bin_pose");
    bin_pose_client_.waitForExistence();
    bin_pose_msgs::bin_pose_vector srv;
    bin_pose_client_.call(srv);

    for (auto &target : srv.response.poses) {
        planner.moveJ(srv.response.poses[0].grasp_pose);
        if (!ros::ok()) break;
    }


  ros::shutdown();
  return 0;
}
