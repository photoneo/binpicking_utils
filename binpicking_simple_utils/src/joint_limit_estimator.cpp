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

#include <binpicking_simple_utils/joint_limit_estimator.h>

JointLimitEstimator::JointLimitEstimator(ros::NodeHandle *nh)
{
  // Initialize Moveit group
  group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

  // Load robot description
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  // Parse joint limits from robot description
  nh->getParam("robot_description", robot_description_);
  urdf_ = urdf::parseURDF(robot_description_);
  joint_limits_.resize(NUM_OF_JOINTS);

  for(int i = 0; i < NUM_OF_JOINTS; i++)
  {
    std::string joint_name = "joint_" + std::to_string(i+1);
    boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(joint_name);
    getJointLimits(joint, joint_limits_[i]);
  }

  marker_pub_ = nh->advertise<visualization_msgs::Marker>("bin_pose_visualization", 1);
}

JointLimitEstimator::~JointLimitEstimator()
{

}

bool JointLimitEstimator::callback(bin_pose_msgs::joint_limits::Request& req, bin_pose_msgs::joint_limits::Response& res)
{
  // Visualize Bin
  visualizeBin(req.bin_corner_1, req.bin_corner_2);

  // Estimate Joint Limits
  double success_rate = estimateJointLimits(req.bin_corner_1, req.bin_corner_2, req.iterations);

  for(int i = 0; i < NUM_OF_JOINTS; i++)
  {
    moveit_msgs::JointLimits current_joint;

    current_joint.joint_name = "joint" + std::to_string(i+1);
    current_joint.has_velocity_limits = joint_limits_[i].has_velocity_limits;
    current_joint.has_acceleration_limits = joint_limits_[i].has_acceleration_limits;
    current_joint.has_position_limits = joint_limits_[i].has_position_limits;
    current_joint.max_velocity = joint_limits_[i].max_velocity;
    current_joint.max_acceleration = joint_limits_[i].max_acceleration;
    current_joint.min_position = joint_limits_[i].min_position;
    current_joint.max_position = joint_limits_[i].max_position;
    res.joint_limits.push_back(current_joint);
  }

  res.success_rate = success_rate;

  return true;

}

double JointLimitEstimator::estimateJointLimits(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2, int iterations)
{
  int success_counter = 0;
  std::vector<double> joint_values;
  std::vector<std::vector<int>> joints_values_multiplicity(NUM_OF_JOINTS, std::vector<int>(NUM_OF_INTERVALS));

  // Initialize kinematic model kinematic state and joint_model_group instances
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  // Acquire data
  for(int i = 0; i < iterations; i++)
  {
    // Get random bin picking pose
    Eigen::Affine3d grasp_state = generateRandomBinPose(bin_corner_1, bin_corner_2);

    // Calculate IK in target pose
    bool found_grasp_ik     = kinematic_state->setFromIK(joint_model_group, grasp_state, IK_ATTEMPTS, IK_TIMEOUT);
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Increment histogram data
    for(int j = 0; j < NUM_OF_JOINTS; j++)
    {
      int index = (int)(joint_values[j] * 10) + (NUM_OF_INTERVALS + 1)/2;
      joints_values_multiplicity[j][index] += 1;
    }

    // Increment IK success counter
    if(found_grasp_ik == true)
      success_counter += 1;
  }

  // Process data
  for(int i = 0; i < NUM_OF_JOINTS; i++)
  {
    int percentage_counter = 0;
    double percentage_value;
    bool min_value_found = false;
    bool max_value_found = false;

    for(int j = 0; j < NUM_OF_INTERVALS; j++)
    {
      percentage_counter += joints_values_multiplicity[i][j];
      percentage_value = (double)percentage_counter / iterations;

      if ((percentage_value > ACCEPTANCE_MIN_THRESHOLD) && (min_value_found == false))
      {
        double index = j;
        joint_limits_[i].min_position = (index/ 10) - (double)(NUM_OF_INTERVALS + 1)/20;
        min_value_found = true;
      }
      if((percentage_value > ACCEPTANCE_MAX_THRESHOLD) && (max_value_found == false))
      {
        double index = j;
        joint_limits_[i].max_position = (index/ 10) - (double)(NUM_OF_INTERVALS + 1)/20;
        max_value_found = true;
      }
    }
  }

  // Return success rate
  return success_counter / (double)iterations;

  // Print data
  //for(int i = 0; i < NUM_OF_JOINTS; i++)
  //{
  //  ROS_INFO("JOINT %d", i+1);
  //  for(int j = 0; j < NUM_OF_INTERVALS; j++)
  //  {
  //    float bottom_range = (j-64)/10.0;
  //    float upper_range = ((j-64)/10.0) + 0.1;
  //    ROS_INFO("Range: [%1.1f - %1.1f]: %d", bottom_range, upper_range , joints_values_multiplicity[i][j]);
  //  }
  //}
}

Eigen::Affine3d JointLimitEstimator::generateRandomBinPose(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2)
{
  // Generate random Grasp pose
  geometry_msgs::Pose grasp_pose;
  Eigen::Affine3d grasp_state;

  if(bin_corner_1.x < bin_corner_2.x)
    grasp_pose.position.x = randGen(bin_corner_1.x, bin_corner_2.x);
  else
    grasp_pose.position.x = randGen(bin_corner_2.x, bin_corner_1.x);

  if(bin_corner_1.y < bin_corner_2.y)
    grasp_pose.position.y = randGen(bin_corner_1.y, bin_corner_2.y);
  else
    grasp_pose.position.y = randGen(bin_corner_2.y, bin_corner_1.y);

  if(bin_corner_1.z < bin_corner_2.z)
    grasp_pose.position.z = randGen(bin_corner_1.z, bin_corner_2.z);
  else
    grasp_pose.position.z = randGen(bin_corner_2.z, bin_corner_1.z);

  double roll = randGen(-ALLOWED_ROLL_RANGE/2, ALLOWED_ROLL_RANGE/2);
  double pitch = randGen(M_PI - ALLOWED_PITCH_RANGE/2, M_PI + ALLOWED_PITCH_RANGE/2);
  double yaw = randGen(-ALLOWED_YAW_RANGE/2, ALLOWED_YAW_RANGE/2);
  grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  // Convert geometry_msgs pose to Eigen representation
  tf::poseMsgToEigen(grasp_pose, grasp_state);

  // Return Eigen::Affine3d representation
  return grasp_state;
}

double JointLimitEstimator::randGen(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void JointLimitEstimator::visualizeBin(geometry_msgs::Point bin_corner_1, geometry_msgs::Point bin_corner_2)
{
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();

  marker.ns = "bin";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (bin_corner_1.x + bin_corner_2.x)/2;
  marker.pose.position.y = (bin_corner_1.y + bin_corner_2.y)/2;
  marker.pose.position.z = (bin_corner_1.z + bin_corner_2.z)/2;

  marker.scale.x = fabs(bin_corner_1.x - bin_corner_2.x);
  marker.scale.y = fabs(bin_corner_1.y - bin_corner_2.y);
  marker.scale.z = fabs(bin_corner_1.z - bin_corner_2.z);

  marker.color.r = 0.8f;
  marker.color.g = 0.0f;
  marker.color.b = 0.8f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_limit_estimator");
  ros::NodeHandle nh;

  // Wait for robot_description param
  while(nh.hasParam("robot_description") == 0)
    {
      ROS_WARN("Waiting for robot_description to appear on parameter server ");
      ros::Duration(0.1).sleep();
  }

  // Create Joint Limit Estimator object
  JointLimitEstimator joint_limit_estimator(&nh);

  // Advertise service
  ros::ServiceServer service = nh.advertiseService("estimate_joint_limits", &JointLimitEstimator::callback, &joint_limit_estimator);

  ros::spin();

  return EXIT_SUCCESS;
}
