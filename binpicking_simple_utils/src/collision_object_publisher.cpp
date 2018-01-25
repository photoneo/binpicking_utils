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

#include <binpicking_simple_utils/collision_object_publisher.h>

CollisionObjectPublisher::CollisionObjectPublisher(ros::NodeHandle *nh, std::string collision_object_list_filepath)
{
  try
  {
    YAML::Node config = YAML::LoadFile(collision_object_list_filepath);

    // Parse single collision object data one by one
    for(std::size_t i = 0; i < config.size(); i++)
    {
      CollisionObject single_object;
      single_object.label            = config[i]["label"].as<std::string>();
      single_object.model_filepath   = config[i]["model_filepath"].as<std::string>();
      single_object.x_position       = config[i]["x_position"].as<double>();
      single_object.y_position       = config[i]["y_position"].as<double>();
      single_object.z_position       = config[i]["z_position"].as<double>();

      single_object.roll             = config[i]["roll"].as<double>();
      single_object.pitch            = config[i]["pitch"].as<double>();
      single_object.yaw              = config[i]["yaw"].as<double>();

      single_object.x_scale          = config[i]["x_scale"].as<double>();
      single_object.y_scale          = config[i]["y_scale"].as<double>();
      single_object.z_scale          = config[i]["z_scale"].as<double>();

      collision_objects.push_back(single_object);
    }
  }
  catch(YAML::ParserException &e)
  {
    ROS_ERROR("Error reading yaml config file! Check list of collision objects file");
    ros::shutdown();
  }

  // Initialize ros::Publisher
  pub = nh->advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  
}


CollisionObjectPublisher::~CollisionObjectPublisher()
{

}


void CollisionObjectPublisher::publishAllCollisionObjects()
{
  for(int i = 0; i < collision_objects.size(); i++)
  {
    ROS_INFO_STREAM("[Label          ]: " << collision_objects[i].label);
    ROS_INFO_STREAM("[Model filepath ]: " << collision_objects[i].model_filepath);
    ROS_INFO_STREAM("[X position     ]: " << collision_objects[i].x_position );
    ROS_INFO_STREAM("[Y position     ]: " << collision_objects[i].y_position );
    ROS_INFO_STREAM("[Z position     ]: " << collision_objects[i].z_position );
    ROS_INFO_STREAM("[ROLL           ]: " << collision_objects[i].roll );
    ROS_INFO_STREAM("[PITCH          ]: " << collision_objects[i].pitch );
    ROS_INFO_STREAM("[YAW            ]: " << collision_objects[i].yaw );
    ROS_INFO_STREAM("[SCALE X        ]: " << collision_objects[i].x_scale );
    ROS_INFO_STREAM("[SCALE Y        ]: " << collision_objects[i].y_scale );
    ROS_INFO_STREAM("[SCALE Z        ]: " << collision_objects[i].z_scale );

    publishSingleCollisionObject(collision_objects[i]);
  }
}

void CollisionObjectPublisher::publishSingleCollisionObject(CollisionObject single_object)
{
  moveit_msgs::CollisionObject collision_object;
  const Eigen::Vector3d mesh_scale(single_object.x_scale, single_object.y_scale, single_object.z_scale);
  shapes::Mesh* mesh = shapes::createMeshFromResource(single_object.model_filepath, mesh_scale);

  shape_msgs::Mesh collision_object_mesh;
  shapes::ShapeMsg collision_object_mesh_msg;
  shapes::constructMsgFromShape(mesh, collision_object_mesh_msg);
  collision_object_mesh = boost::get<shape_msgs::Mesh>(collision_object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = collision_object_mesh;
  collision_object.header.frame_id = "base_link";
  collision_object.id = single_object.label;

  collision_object.mesh_poses[0].position.x = single_object.x_position;
  collision_object.mesh_poses[0].position.y = single_object.y_position;
  collision_object.mesh_poses[0].position.z = single_object.z_position;

  tf::Quaternion quaternion;
  quaternion.setRPY(single_object.roll, single_object.pitch, single_object.yaw);

  collision_object.mesh_poses[0].orientation.w= quaternion.getW();
  collision_object.mesh_poses[0].orientation.x= quaternion.getX();
  collision_object.mesh_poses[0].orientation.y= quaternion.getY();
  collision_object.mesh_poses[0].orientation.z= quaternion.getZ();

  collision_object.meshes.push_back(collision_object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;

  pub.publish(collision_object);
  ROS_INFO_STREAM("Collision object[ " << single_object.label << " ] published");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_object_publisher");
  ros::NodeHandle nh;

  // Get collision object list filepath
  std::string collision_objects_list_filepath;
  nh.getParam("collision_objects_list_filepath", collision_objects_list_filepath);

  CollisionObjectPublisher collision_object_publisher(&nh, collision_objects_list_filepath);
  
  ros::Duration(5).sleep();
    
  collision_object_publisher.publishAllCollisionObjects();

  ros::shutdown();
  return EXIT_SUCCESS;
}
