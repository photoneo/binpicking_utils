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

#include <binpicking_emulator/binpicking_emulator.h>

#include <eigen_conversions/eigen_msg.h>




BinpickingEmulator::BinpickingEmulator(ros::NodeHandle& nh) :
    move_group_(PLANNING_GROUP),
    kinematic_(robot_model_loader::RobotModelLoader("robot_description").getModel())
    {
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

move_group_.setPlanningTime(10000);

}

BinpickingEmulator::~BinpickingEmulator(){
}

void BinpickingEmulator::setStartState(const JointValues &start_state) {
    robot_state::RobotStatePtr current_state = move_group_.getCurrentState();
    current_state->setJointGroupPositions("manipulator", start_state);
    move_group_.setStartState(*current_state);
}

void BinpickingEmulator::setConstrains() {
    robot_state::RobotStatePtr current_state = move_group_.getCurrentState();
    const Eigen::Affine3d &end_effector_state = current_state->getGlobalLinkTransform("tool1");

    geometry_msgs::Pose pose;
    pose.position.x = end_effector_state.translation()[0];
    pose.position.y = end_effector_state.translation()[1];
    pose.position.z = end_effector_state.translation()[2];

    auto rotation_matrix = end_effector_state.rotation();
    // convert rotation matrix to tf matrix
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(rotation_matrix(0, 0)), static_cast<double>(rotation_matrix(0, 1)),
                  static_cast<double>(rotation_matrix(0, 2)),
                  static_cast<double>(rotation_matrix(1, 0)), static_cast<double>(rotation_matrix(1, 1)),
                  static_cast<double>(rotation_matrix(1, 2)),
                  static_cast<double>(rotation_matrix(2, 0)), static_cast<double>(rotation_matrix(2, 1)),
                  static_cast<double>(rotation_matrix(2, 2)));

    // Convert to quternion
    tf::Quaternion quaternion;
    tf3d.getRotation(quaternion);

    moveit_msgs::OrientationConstraint ocm;
//    ocm.orientation.x = quaternion.x();
//    ocm.orientation.y = quaternion.y();
//    ocm.orientation.z = quaternion.z();
//    ocm.orientation.w = quaternion.w();
    ocm.orientation.x = 0.001005;
    ocm.orientation.y = 0.984702;
    ocm.orientation.z = 0.00078;
    ocm.orientation.w = 0.174239;

    ocm.link_name = "tool1";
    ocm.header.frame_id = "base_link";
  //  ocm.orientation = pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_.setPathConstraints(test_constraints);
}

BinpickingEmulator::Result BinpickingEmulator::moveJ(const JointValues &joint_pose, trajectory_msgs::JointTrajectory &trajectory) {
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_.setJointValueTarget(joint_pose);
    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    if (success) {
        trajectory = my_plan.trajectory_.joint_trajectory;
        return Result::OK;
    } else {
        return Result::PATH_PLAN_FAIL;
    }
}

BinpickingEmulator::Result BinpickingEmulator::moveJ(const geometry_msgs::Pose &pose, trajectory_msgs::JointTrajectory &trajectory) {
    JointValues joint_values;
    if (!kinematic_.getIK(pose, joint_values)) {
        return Result::IK_FAIL;
    }
    return moveJ(joint_values, trajectory);
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
