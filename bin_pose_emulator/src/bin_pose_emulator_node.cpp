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
//#include "bin_pose_emulator/ActionServerInterface.h"

//#define RANDOM_BIN_POSE


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bin_pose_emulator");
  ros::NodeHandle nh;

  // Get config filepath from ROS Param server
  std::string filepath;
  nh.getParam("filepath", filepath);

  // Create emulator object
  BinPoseEmulator emulator(&nh, filepath);

  //ActionServerInterface actionServer(std::make_shared<BinPoseEmulator>(emulator));
  // Advertise service
  //ros::ServiceServer service =
  //    nh.advertiseService("bin_pose", &BinPoseEmulator::callback, &emulator);
    ros::ServiceServer service =
            nh.advertiseService("bin_pose", &BinPoseEmulator::callbackVect, &emulator);
  ROS_WARN("creating service");
  ros::spin();

  return EXIT_SUCCESS;
}
