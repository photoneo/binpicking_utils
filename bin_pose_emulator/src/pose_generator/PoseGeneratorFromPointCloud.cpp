//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloud.h"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

PoseGeneratorFromPointCloud::PoseGeneratorFromPointCloud(ros::NodeHandle &nh) : PoseGeneratorBase(nh){
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("bin_pose_point_cloud", 1);
}

bool PoseGeneratorFromPointCloud::generate(geometry_msgs::Pose &pose){

    static int iterator = 0;

    static double roll = config_.roll_default -config_.roll_range / 2;
    static double pitch = config_.pitch_default -config_.pitch_range / 2;
    static double yaw = config_.yaw_default -config_.yaw_range / 2;

    yaw += config_.step_yaw;
    if (yaw >= config_.yaw_default + config_.yaw_range / 2) {
        yaw = config_.yaw_default - config_.yaw_range / 2;
        pitch += config_.step_pitch;

        if (pitch >= config_.pitch_default + config_.pitch_range / 2) {
            pitch = config_.pitch_default - config_.pitch_range / 2;
            roll += config_.step_roll;

            if (roll >= config_.roll_default + config_.roll_range / 2) {
                roll = config_.roll_default - config_.roll_range / 2;
                iterator++;
            }
        }
    }

    if (iterator >= space->points.size())
        return false;

    pose.position.x = space->points[iterator].x;
    pose.position.y = space->points[iterator].y;
    pose.position.z = space->points[iterator].z;
    tf::Quaternion grasp_orientation;
    grasp_orientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = grasp_orientation.getX();
    pose.orientation.y = grasp_orientation.getY();
    pose.orientation.z = grasp_orientation.getZ();
    pose.orientation.w = grasp_orientation.getW();
    return true;
}

bool PoseGeneratorFromPointCloud::parseConfig(std::string filepath) {

    try {
        YAML::Node config_file = YAML::LoadFile(filepath);

        config_.roll_default = config_file["roll_default"].as<float>();
        config_.pitch_default = config_file["pitch_default"].as<float>();
        config_.yaw_default = config_file["yaw_default"].as<float>();
        config_.roll_range = config_file["roll_range"].as<float>();
        config_.pitch_range = config_file["pitch_range"].as<float>();
        config_.yaw_range = config_file["yaw_range"].as<float>();
        config_.step_roll = config_file["step_roll"].as<float>();
        config_.step_pitch = config_file["step_pitch"].as<float>();
        config_.step_yaw = config_file["step_yaw"].as<float>();
        config_.mesh = config_file["mesh"].as<std::string>();
    }
    catch (YAML::ParserException &e) {
        ROS_ERROR("Bin pose emulator: Error reading yaml config file");
    }

    // Load Ply picked object into point cloud
    space = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());

    pcl::PLYReader plyReader;
    plyReader.read(config_.mesh, *space);
    if (!space) {
        ROS_ERROR("Unable to open file");
        return false;
    }

    for (auto &point : *space){
        point.x /= 1000;
        point.y /= 1000;
        point.z /= 1000;
    }

    pcl::toROSMsg(*space.get(),point_cloud_msg_ );
    point_cloud_msg_.header.frame_id = "base_link";

    return true;
}

void PoseGeneratorFromPointCloud::visualizeBin()
{
    point_cloud_msg_.header.stamp = ros::Time::now();
    point_cloud_pub_.publish(point_cloud_msg_);
}