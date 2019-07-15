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

#ifndef BIN_POSE_EMULATOR_H
#define BIN_POSE_EMULATOR_H

#include <bin_pose_emulator/ActionServerInterface.h>
#include <bin_pose_msgs/bin_pose.h>

class BinPoseEmulator : public ActionServerInterface
{
public:
  BinPoseEmulator(ros::NodeHandle &nh, std::string filepath);
  ~BinPoseEmulator();

  bool callback(bin_pose_msgs::bin_pose::Request& req,
                bin_pose_msgs::bin_pose::Response& res);

protected:

private:
    ros::ServiceServer service;
};

#endif // BIN_POSE_EMULATOR_H
