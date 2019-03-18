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

#include <ros/ros.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <bin_pose_msgs/bin_pose.h>


struct ConfigData
{
  // Virtual Bin center
  double bin_center_x;
  double bin_center_y;
  double bin_center_z;

  // Virtual Bin size
  double bin_size_x;
  double bin_size_y;
  double bin_size_z;

    // Virtual Bin rotation around world Z axis
  double x_rotation;
  double y_rotation;
  double z_rotation;

  // Default tool point orientation
  double roll_default;
  double pitch_default;
  double yaw_default;

  // Allowed orientation range
  double roll_range;
  double pitch_range;
  double yaw_range;

  // Planning constraints
  double approach_distance;
  double deapproach_height;

  double step_x;
  double step_y;
  double step_z;
  double step_roll;
  double step_pitch;
  double step_yaw;
};

class BinPoseEmulator
{
public:
  BinPoseEmulator(ros::NodeHandle* nh, std::string filepath);
  ~BinPoseEmulator();

  bool callback(bin_pose_msgs::bin_pose::Request& req,
                bin_pose_msgs::bin_pose::Response& res);
  bool getPose(geometry_msgs::Pose &pose, bool is_random = true);
    bool getNextPose(geometry_msgs::Pose &pose);
    void getRandomPose(geometry_msgs::Pose &pose);

protected:
    bool parseConfig(std::string filepath);
    tf::Transform visualizeBin(void);
    void visualizePose(geometry_msgs::Pose grasp_pose,
                       geometry_msgs::Pose approach_pose, bool multiArray = false);
    tf::Transform broadcastPoseTF(geometry_msgs::Pose grasp_pose);

private:
  double randGen(double fMin, double fMax);



  ros::Publisher marker_pub_;

  ConfigData config_;

  tf::TransformBroadcaster broadcaster_;


};

#endif // BIN_POSE_EMULATOR_H
