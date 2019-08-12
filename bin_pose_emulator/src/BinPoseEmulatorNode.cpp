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

#include "bin_pose_emulator/BinPoseEmulator.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bin_pose_emulator");

  // Create emulator object
  ros::NodeHandle nh("vision_system_1");
  BinPoseEmulator emulator(nh);

  ros::spin();

  return EXIT_SUCCESS;
}
