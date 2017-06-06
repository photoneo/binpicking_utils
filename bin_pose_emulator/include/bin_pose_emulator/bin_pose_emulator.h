/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Photoneo s.r.o.
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
 *   * Neither the name of the Photoneo nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
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
};

class Emulator
{
public:
  Emulator(ros::NodeHandle* nh, std::string filepath);
  ~Emulator();

  bool callback(bin_pose_msgs::bin_pose::Request& req,
                bin_pose_msgs::bin_pose::Response& res);

private:
  double randGen(double fMin, double fMax);
  bool parseConfig(std::string filepath);

  void visualize_bin(void);
  void visualize_pose(geometry_msgs::Pose grasp_pose,
                      geometry_msgs::Pose approach_pose);
  void broadcast_pose_tf(geometry_msgs::Pose grasp_pose);

  ros::Publisher marker_pub;

  ConfigData config;
};

#endif // BIN_POSE_EMULATOR_H
