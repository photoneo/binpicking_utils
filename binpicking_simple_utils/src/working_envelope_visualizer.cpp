/*********************************************************************
Copyright [2018] [Frantisek Durovsky]

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

#include <binpicking_simple_utils/working_envelope_visualizer.h>

WorkingEnvelopeVisualizer::WorkingEnvelopeVisualizer(ros::NodeHandle *nh)
{
  // Initialize Moveit group
  group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

  // Load robot description
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  // Parse joint limits from robot description
  //nh->getParam("robot_description", robot_description_);
  //urdf_ = urdf::parseURDF(robot_description_);
  //joint_limits_.resize(NUM_OF_JOINTS);
  //
  //for(int i = 0; i < NUM_OF_JOINTS; i++)
  //{
  //  std::string joint_name = "joint_" + std::to_string(i+1);
  //  boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(joint_name);
  //  getJointLimits(joint, joint_limits_[i]);
  //}
  //

  point_pub_ = nh->advertise<visualization_msgs::Marker>("points", 1000);
  point_cloud_pub_ = nh->advertise<pcl::PointCloud<pcl::PointXYZ>>("pointcloud", 1000);
  point_id_counter_ = 0;

  joint_limit_client_ = nh->serviceClient<bin_pose_msgs::joint_limits>("estimate_joint_limits");
}

WorkingEnvelopeVisualizer::~WorkingEnvelopeVisualizer()
{

}

bool WorkingEnvelopeVisualizer::callback(bin_pose_msgs::envelope_visualize::Request& req, bin_pose_msgs::envelope_visualize::Response& res)
{
  // Initialize kinematic model kinematic state and joint_model_group instances
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  // Initialize PCL variables
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->header.frame_id = "base_link";
  cloud->height = cloud->width = 1;

  // Initialize kinematic state
  kinematic_state->setToDefaultValues();
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  int points_calculated = 0;

  // Get Joint Limits
  std::vector<moveit_msgs::JointLimits> joint_limits;
  geometry_msgs::Point bin_corner_1, bin_corner_2;
  bin_corner_1.x = 0.7;
  bin_corner_1.y = -0.8;
  bin_corner_1.z = 0.0;
  bin_corner_2.x = 1.7;
  bin_corner_2.y = 0.8;
  bin_corner_2.z = 1.0;

  //visualizePoint(++point_id_counter_, 0.1, bin_corner_1.x, bin_corner_1.y, bin_corner_1.z);
  //visualizePoint(++point_id_counter_, 0.1, bin_corner_2.x, bin_corner_2.y, bin_corner_2.z);

  bin_pose_msgs::joint_limits srv;
  srv.request.bin_corner_1 = req.bin_corner_1;
  srv.request.bin_corner_2 = req.bin_corner_2;
  srv.request.iterations = 1000;

  if (joint_limit_client_.call(srv))
  {
    for(int i = 0; i < srv.response.joint_limits.size(); i++)
    {
      moveit_msgs::JointLimits temp_joint;
      temp_joint = srv.response.joint_limits[i];
      ROS_INFO("Joint %d: [%f, %f]", i+1, srv.response.joint_limits[i].min_position, srv.response.joint_limits[i].max_position);
      joint_limits.push_back(temp_joint);
    }

  }
  else
  {
    ROS_ERROR("Failed to call service estimate_joint_limits");
    return 1;
  }

  double joint_1_increment = (joint_limits[0].max_position - joint_limits[0].min_position) / 10;
  double joint_2_increment = (joint_limits[1].max_position - joint_limits[1].min_position) / 10;
  double joint_3_increment = (joint_limits[2].max_position - joint_limits[2].min_position) / 10;
  double joint_4_increment = (joint_limits[3].max_position - joint_limits[3].min_position) / 5;
  double joint_5_increment = (joint_limits[4].max_position - joint_limits[4].min_position) / 5;
  double joint_6_increment = (joint_limits[5].max_position - joint_limits[5].min_position) / 5;

  for(double joint_1_position = joint_limits[0].min_position; joint_1_position <= joint_limits[0].max_position; joint_1_position += joint_1_increment )
  {
    kinematic_state->setJointPositions("joint_1", &joint_1_position);

    for(double joint_2_position = joint_limits[1].min_position; joint_2_position <= joint_limits[1].max_position; joint_2_position += joint_2_increment )
    {
      kinematic_state->setJointPositions("joint_2", &joint_2_position);

      for(double joint_3_position = joint_limits[2].min_position; joint_3_position <= joint_limits[2].max_position; joint_3_position += joint_3_increment )
      {
        kinematic_state->setJointPositions("joint_3", &joint_3_position);

        for(double joint_4_position = joint_limits[3].min_position; joint_4_position <= joint_limits[3].max_position; joint_4_position += joint_4_increment)
        {
          kinematic_state->setJointPositions("joint_4", &joint_4_position);

          for(double joint_5_position = joint_limits[4].min_position; joint_5_position <= joint_limits[4].max_position; joint_5_position += joint_5_increment)
          {
            kinematic_state->setJointPositions("joint_5", &joint_5_position);

            for(double joint_6_position = joint_limits[5].min_position; joint_6_position <= joint_limits[5].max_position; joint_6_position += joint_6_increment)
            {
              kinematic_state->setJointPositions("joint_6", &joint_6_position);
              const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool1");
              points_calculated++;

              // Conversions
              pcl::PointXYZ single_pcl_point;
              geometry_msgs::Pose single_geometry_msgs_point;
              tf::poseEigenToMsg(end_effector_state, single_geometry_msgs_point);

              single_pcl_point.x = single_geometry_msgs_point.position.x;
              single_pcl_point.y = single_geometry_msgs_point.position.y;
              single_pcl_point.z = single_geometry_msgs_point.position.z;

              cloud->push_back(single_pcl_point);
            }
          }
        }
      }
    }
  }

  pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
  point_cloud_pub_.publish(cloud);

  std::string writePath = "/home/controller/catkin_ws/point_cloud.ply";
  pcl::io::savePLYFileBinary(writePath, *cloud);

  ROS_INFO("Points calculated: %d", points_calculated);

  //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  //viewer.showCloud (cloud);
  //while (!viewer.wasStopped ())
  //{
  //}

  res.success = true;
  return true;
}

void WorkingEnvelopeVisualizer::visualizePoint(int id, double marker_size, double x, double y, double z)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/base_link";
  marker.ns = "points";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.header.stamp = ros::Time::now();
  marker.id = id;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  marker.scale.x = marker_size;
  marker.scale.y = marker_size;
  marker.scale.z = marker_size;

  marker.color.r = 0.9f;
  marker.color.g = 0.9f;
  marker.color.b = 0.9f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(60);
  point_pub_.publish(marker);
  ros::Duration(0.0001).sleep();

}

void WorkingEnvelopeVisualizer::visualizePose(int id, geometry_msgs::Pose pose)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/base_link";
  marker.ns = "arrows";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.header.stamp = ros::Time::now();
  marker.id = id;

  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;

  marker.pose.orientation.x = pose.orientation.x;
  marker.pose.orientation.y = pose.orientation.y;
  marker.pose.orientation.z = pose.orientation.z;
  marker.pose.orientation.w = pose.orientation.w;

  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.05;

  marker.color.r = 0.9f;
  marker.color.g = 0.9f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(30);
  point_pub_.publish(marker);
  ros::Duration(0.001).sleep();

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "working_envelope_visualizer");
  ros::NodeHandle nh;

  // Wait for robot_description param
  while(nh.hasParam("robot_description") == 0)
    {
      ROS_WARN("Waiting for robot_description to appear on parameter server ");
      ros::Duration(0.1).sleep();
  }

  // Create Working Envelope Visualizer object
  WorkingEnvelopeVisualizer visualizer(&nh);

  // Advertise service
  ros::ServiceServer service = nh.advertiseService("working_envelope_visualizer", &WorkingEnvelopeVisualizer::callback, &visualizer);

  ros::spin();

  return EXIT_SUCCESS;
}
