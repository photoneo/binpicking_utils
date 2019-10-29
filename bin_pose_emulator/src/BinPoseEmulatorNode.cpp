#include "bin_pose_emulator/BinPoseEmulator.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bin_pose_emulator");

  // Create emulator object
  ros::NodeHandle nh("vision_system_1");
  BinPoseEmulator emulator(nh);

  ros::spin();

  return EXIT_SUCCESS;
}
