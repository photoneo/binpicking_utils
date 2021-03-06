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

BinPoseEmulator::BinPoseEmulator(ros::NodeHandle* nh, std::string filepath)
{
  parseConfig(filepath); // parse yaml config file
  srandom(time(NULL));   // initialize random generator

  marker_pub_ =
      nh->advertise<visualization_msgs::Marker>("bin_pose_visualization", 1);

  ROS_WARN("BIN POSE EMULATOR: Ready!");
}

BinPoseEmulator::~BinPoseEmulator() {}

bool BinPoseEmulator::callback(bin_pose_msgs::bin_pose::Request& req,
                        bin_pose_msgs::bin_pose::Response& res)
{

  //-----------------------------------------------------------------------------------------
  // Generate random Grasp pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position.x = randGen(config_.bin_center_x - config_.bin_size_x / 2,
                                  config_.bin_center_x + config_.bin_size_x / 2);
  grasp_pose.position.y = randGen(config_.bin_center_y - config_.bin_size_y / 2,
                                  config_.bin_center_y + config_.bin_size_y / 2);
  grasp_pose.position.z = randGen(config_.bin_center_z - config_.bin_size_z / 2,
                                  config_.bin_center_z + config_.bin_size_z / 2);

  double grasp_roll = randGen(config_.roll_default - config_.roll_range / 2,
                              config_.roll_default + config_.roll_range / 2);
  double grasp_pitch = randGen(config_.pitch_default - config_.pitch_range / 2,
                               config_.pitch_default + config_.pitch_range / 2);
  double grasp_yaw = randGen(config_.yaw_default - config_.yaw_range / 2,
                             config_.yaw_default + config_.yaw_range / 2);

  tf::Quaternion grasp_orientation;
  grasp_orientation.setRPY(grasp_roll, grasp_pitch, grasp_yaw);
  grasp_pose.orientation.x = grasp_orientation.getX();
  grasp_pose.orientation.y = grasp_orientation.getY();
  grasp_pose.orientation.z = grasp_orientation.getZ();
  grasp_pose.orientation.w = grasp_orientation.getW();

  res.grasp_pose = grasp_pose;

  //------------------------------------------------------------------------------------------
  // Calculate Approach pose according to existing grasp pose
  geometry_msgs::Pose approach_pose;

  tf::Vector3 vector(0, 0, 1);
  tf::Vector3 rotated_vector = tf::quatRotate(grasp_orientation, vector);

  approach_pose.position.x =
      grasp_pose.position.x - config_.approach_distance * rotated_vector.getX();
  approach_pose.position.y =
      grasp_pose.position.y - config_.approach_distance * rotated_vector.getY();
  approach_pose.position.z =
      grasp_pose.position.z - config_.approach_distance * rotated_vector.getZ();

  approach_pose.orientation = grasp_pose.orientation;
  res.approach_pose = approach_pose;

  //-------------------------------------------------------------------------------------------
  // Calculate Deapproach point according to exitsing grasp pose
  geometry_msgs::Pose deapproach_pose;

  deapproach_pose = grasp_pose;
  deapproach_pose.position.z =
      deapproach_pose.position.z + config_.deapproach_height;

  visualizeBin();
  visualizePose(grasp_pose, approach_pose);
  broadcastPoseTF(grasp_pose);

  res.deapproach_pose = deapproach_pose;
  
  return true;
}

double BinPoseEmulator::randGen(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

bool BinPoseEmulator::parseConfig(std::string filepath)
{
  try
  {
    YAML::Node config_file = YAML::LoadFile(filepath);
    config_.bin_center_x = config_file["bin_center_x"].as<float>();
    config_.bin_center_y = config_file["bin_center_y"].as<float>();
    config_.bin_center_z = config_file["bin_center_z"].as<float>();

    config_.bin_size_x = config_file["bin_size_x"].as<float>();
    config_.bin_size_y = config_file["bin_size_y"].as<float>();
    config_.bin_size_z = config_file["bin_size_z"].as<float>();

    config_.roll_default = config_file["roll_default"].as<float>();
    config_.pitch_default = config_file["pitch_default"].as<float>();
    config_.yaw_default = config_file["yaw_default"].as<float>();

    config_.roll_range = config_file["roll_range"].as<float>();
    config_.pitch_range = config_file["pitch_range"].as<float>();
    config_.yaw_range = config_file["yaw_range"].as<float>();

    config_.approach_distance = config_file["approach_distance"].as<float>();
    config_.deapproach_height = config_file["deapproach_height"].as<float>();
  }
  catch (YAML::ParserException& e)
  {
    ROS_ERROR("Bin pose emulator: Error reading yaml config file");
  }
}

void BinPoseEmulator::visualizeBin(void)
{
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();

  marker.ns = "bin";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = config_.bin_center_x;
  marker.pose.position.y = config_.bin_center_y;
  marker.pose.position.z = config_.bin_center_z;

  marker.scale.x = config_.bin_size_x;
  marker.scale.y = config_.bin_size_y;
  marker.scale.z = config_.bin_size_z;

  marker.color.r = 0.8f;
  marker.color.g = 0.0f;
  marker.color.b = 0.8f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);
}

void BinPoseEmulator::visualizePose(geometry_msgs::Pose grasp_pose,
                              geometry_msgs::Pose approach_pose)
{
  uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();

  marker.ns = "bin";
  marker.id = 1;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point approach_point;
  approach_point.x = approach_pose.position.x;
  approach_point.y = approach_pose.position.y;
  approach_point.z = approach_pose.position.z;

  geometry_msgs::Point grasp_point;
  grasp_point.x = grasp_pose.position.x;
  grasp_point.y = grasp_pose.position.y;
  grasp_point.z = grasp_pose.position.z;

  marker.points.push_back(approach_point);
  marker.points.push_back(grasp_point);

  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.05;

  marker.color.r = 0.9f;
  marker.color.g = 0.9f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);
}

void BinPoseEmulator::broadcastPoseTF(geometry_msgs::Pose grasp_pose)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(grasp_pose.position.x, grasp_pose.position.y,
                                  grasp_pose.position.z));
  transform.setRotation(
      tf::Quaternion(grasp_pose.orientation.x, grasp_pose.orientation.y,
                     grasp_pose.orientation.z, grasp_pose.orientation.w));

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                        "base_link", "current_goal"));
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bin_pose_emulator");
  ros::NodeHandle nh;

  // Get config filepath from ROS Param server
  std::string filepath;
  nh.getParam("filepath", filepath);

  // Create emulator object
  BinPoseEmulator emulator(&nh, filepath);

  // Advertise service
  ros::ServiceServer service =
      nh.advertiseService("bin_pose", &BinPoseEmulator::callback, &emulator);

  ros::spin();

  return EXIT_SUCCESS;
}
