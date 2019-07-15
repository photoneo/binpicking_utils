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

#include "bin_pose_emulator/bin_pose_emulator.h"


BinPoseEmulator::BinPoseEmulator(ros::NodeHandle &nh, std::string filepath) : ActionServerInterface(nh, filepath)
{
  ROS_WARN("BIN POSE EMULATOR: Ready!");
    // Advertise service
    service = nh.advertiseService("bin_pose", &BinPoseEmulator::callback, this);
}

BinPoseEmulator::~BinPoseEmulator() {}

bool BinPoseEmulator::callback(bin_pose_msgs::bin_pose::Request& req,
                        bin_pose_msgs::bin_pose::Response& res)
{

  //-----------------------------------------------------------------------------------------
  // Generate random Grasp pose
    pose_generator_->getPose(res.grasp_pose, req.approach_distance);

  //------------------------------------------------------------------------------------------
  // Calculate Approach pose according to existing grasp pose

  tf::Vector3 vector(0, 0, 1);
  tf::Quaternion grasp_orientation(res.grasp_pose.orientation.x,res.grasp_pose.orientation.y, res.grasp_pose.orientation.z, res.grasp_pose.orientation.w);
  tf::Vector3 rotated_vector = tf::quatRotate(grasp_orientation, vector);

  res.approach_pose.position.x =
          res.grasp_pose.position.x - req.approach_distance * rotated_vector.getX();
  res.approach_pose.position.y =
          res.grasp_pose.position.y - req.approach_distance * rotated_vector.getY();
  res.approach_pose.position.z =
          res.grasp_pose.position.z - req.approach_distance * rotated_vector.getZ();

  res.approach_pose.orientation = res.grasp_pose.orientation;

  //-------------------------------------------------------------------------------------------
  // Calculate Deapproach point according to exitsing grasp pose

  res.deapproach_pose = res.grasp_pose;
  res.deapproach_pose.position.z =
          res.deapproach_pose.position.z + req.deaproach_height;

  return true;
}
