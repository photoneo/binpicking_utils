/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Photoneo s.r.o.
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

#include <localization_interface/localization_interface.h>

LocalizationNode::LocalizationNode(ros::NodeHandle* nh)
{
  // Get Localization Params From Param server and check result
  bool get_plcf_success = nh->getParam("localization/plcf_filepath", plcf_filepath_);
  bool get_scanner_id_success = nh->getParam("localization/scanner_id", scanner_id_);

  if((get_plcf_success) && (get_scanner_id_success))
    ROS_INFO("LOCALIZATION NODE: Initializing localization pipeline!");
  else
  {
    ROS_ERROR("LOCALIZATION NODE: Failed to load params from parameter server! Check localization_params.yaml file");
    ros::shutdown();
  }

  // Initialize localization
  try
  {
    Localization.reset(new pho::sdk::PhoLocalization());
  }
  catch(pho::sdk::AuthenticationException ex)
  {
    ROS_ERROR_STREAM("LOCALIZATION NODE: Localization Initialization Failed: " << ex.what());
    ros::shutdown();
  }

  // Connect to PhoXi Scanner
  scanner = factory.CreateAndConnect(scanner_id_.c_str());

  // Set Scene Source
  pho::sdk::SceneSource Scene = pho::sdk::SceneSource::PhoXi(scanner);
  Localization->SetSceneSource(Scene);

  // Load localization configuration
  if(!Localization || !Localization->LoadLocalizationConfiguration(plcf_filepath_.c_str()))
  {
    ROS_ERROR("LOCALIZATION NODE: PLCF file not found, plcf path: %s", plcf_filepath_.c_str());
    ros::shutdown();
  }
}

LocalizationNode::~LocalizationNode()
{

}

bool LocalizationNode::localizationCallback(localization_interface::get_position::Request& req, localization_interface::get_position::Response& res)
{
  double overlap;
  bool object_found = false;
  pho::sdk::LocalizationPose localizationPose;
  pho::sdk::TransformationMatrix4x4* localization_result;

  // Launch localization with acquired frame
  auto AsyncQueue = Localization->StartAsync();

  while (AsyncQueue.GetNext(localizationPose))
  {
    overlap = localizationPose.VisibleOverlap;
    localization_result = new pho::sdk::TransformationMatrix4x4(localizationPose.Transformation);
    object_found = true;
  }
  Localization->StopAsync();

  // Process results
  if(object_found == true)
  {
    tf::Matrix3x3 orientation_matrix;

    // Get Quaternion
    orientation_matrix[0][0] =  localization_result[0][0][0];
    orientation_matrix[0][1] =  localization_result[0][0][1];
    orientation_matrix[0][2] =  localization_result[0][0][2];

    orientation_matrix[1][0] =  localization_result[0][1][0];
    orientation_matrix[1][1] =  localization_result[0][1][1];
    orientation_matrix[1][2] =  localization_result[0][1][2];

    orientation_matrix[2][0] =  localization_result[0][2][0];
    orientation_matrix[2][1] =  localization_result[0][2][1];
    orientation_matrix[2][2] =  localization_result[0][2][2];

    tf::Quaternion quat_orientation;
    orientation_matrix.getRotation(quat_orientation);

    // Convert to Euler Angles
    double roll, pitch, yaw;
    orientation_matrix.getRPY(roll, pitch, yaw);

    // Convert from radians to degrees
    roll = roll * RAD2DEG;
    pitch = pitch * RAD2DEG;
    yaw = yaw * RAD2DEG;

    // Console output
    ROS_INFO("LOCALIZATION NODE: Object Found!");
    ROS_INFO("LOCALIZATION NODE: Position [%f, %f, %f]", localization_result[0][0][3]/1000, localization_result[0][1][3]/1000, localization_result[0][2][3]/1000);
    ROS_INFO("LOCALIZATION NODE: Orientation [%f, %f, %f]", roll, pitch, yaw);

    // Broadcast TF to visualize object position in RViz
    tf::Transform transform;
    tf::Vector3 origin;
    tf::Quaternion orientation;

    origin.setX(localization_result[0][0][3]/1000);
    origin.setY(localization_result[0][1][3]/1000);
    origin.setZ(localization_result[0][2][3]/1000);

    orientation.setX(quat_orientation.getX());
    orientation.setY(quat_orientation.getY());
    orientation.setZ(quat_orientation.getZ());
    orientation.setW(quat_orientation.getW());

    transform.setOrigin(origin);
    transform.setRotation(orientation);

    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "object_pose"));

    // Calculate Approach pose according to existing grasp pose
    geometry_msgs::Pose approach_pose;

    tf::Vector3 vector(0, 0, 1);
    tf::Vector3 rotated_vector = tf::quatRotate(orientation, vector);

    approach_pose.position.x = origin.getX() - APPROACH_HEIGHT * rotated_vector.getX();
    approach_pose.position.y = origin.getY() - APPROACH_HEIGHT * rotated_vector.getY();
    approach_pose.position.z = origin.getZ() - APPROACH_HEIGHT * rotated_vector.getZ();

    approach_pose.orientation.x = orientation.getX();
    approach_pose.orientation.y = orientation.getY();
    approach_pose.orientation.z = orientation.getZ();
    approach_pose.orientation.w = orientation.getW();

    // Service Response
    res.header.stamp = ros::Time::now();

    res.grasp_pose.position.x = localization_result[0][0][3]/1000;
    res.grasp_pose.position.y = localization_result[0][1][3]/1000;
    res.grasp_pose.position.z = localization_result[0][2][3]/1000;

    res.grasp_pose.orientation.x = quat_orientation.getX();
    res.grasp_pose.orientation.y = quat_orientation.getY();
    res.grasp_pose.orientation.z = quat_orientation.getZ();
    res.grasp_pose.orientation.w = quat_orientation.getW();

    res.approach_pose.position.x = approach_pose.position.x;
    res.approach_pose.position.y = approach_pose.position.y;
    res.approach_pose.position.z = approach_pose.position.z;

    res.approach_pose.orientation.x = quat_orientation.getX();
    res.approach_pose.orientation.y = quat_orientation.getY();
    res.approach_pose.orientation.z = quat_orientation.getZ();
    res.approach_pose.orientation.w = quat_orientation.getW();

    res.deapproach_pose.position.x = localization_result[0][0][3]/1000;
    res.deapproach_pose.position.y = localization_result[0][1][3]/1000;
    res.deapproach_pose.position.z = localization_result[0][2][3]/1000 + DEAPPROACH_HEIGHT; //mm

    res.deapproach_pose.orientation.x = quat_orientation.getX();
    res.deapproach_pose.orientation.y = quat_orientation.getY();
    res.deapproach_pose.orientation.z = quat_orientation.getZ();
    res.deapproach_pose.orientation.w = quat_orientation.getW();


    res.localization_status = LOCALIZATION_OK;
  }
  else
  {
    ROS_WARN("Localization Timeout");

    res.header.stamp = ros::Time::now();

    res.grasp_pose.position.x = 0;
    res.grasp_pose.position.y = 0;
    res.grasp_pose.position.z = 0;

    res.grasp_pose.orientation.x = 0;
    res.grasp_pose.orientation.y = 0;
    res.grasp_pose.orientation.z = 0;
    res.grasp_pose.orientation.w = 1;

    res.approach_pose.position.x = 0;
    res.approach_pose.position.y = 0;
    res.approach_pose.position.z = 0;

    res.approach_pose.orientation.x = 0;
    res.approach_pose.orientation.y = 0;
    res.approach_pose.orientation.z = 0;
    res.approach_pose.orientation.w = 1;

    res.deapproach_pose.position.x = 0;
    res.deapproach_pose.position.y = 0;
    res.deapproach_pose.position.z = 0;

    res.deapproach_pose.orientation.x = 0;
    res.deapproach_pose.orientation.y = 0;
    res.deapproach_pose.orientation.z = 0;
    res.deapproach_pose.orientation.w = 1;

    res.localization_status = LOCALIZATION_TIMEOUT;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_node");
  ros::NodeHandle nh;

  // Wait for localization/scanner_id param
  while(nh.hasParam("localization/scanner_id") == 0)
  {
    ROS_WARN("LOCALIZATION NODE: Waiting for localization/scanner_id parameter");
    ros::Duration(5).sleep();
  }

  // Create LocalizationNode instance
  LocalizationNode ln(&nh);

  // Advertise service
  ros::ServiceServer localization_service = nh.advertiseService("localization/get_position", &LocalizationNode::localizationCallback, &ln);

  ROS_WARN("LOCALIZATION NODE: Ready");

  // Start Async Spinner
  ros::spin();

  return EXIT_SUCCESS;
}
