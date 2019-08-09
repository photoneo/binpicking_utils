//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloud.h"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <bin_pose_emulator/utils.h>

PoseGeneratorFromPointCloud::PoseGeneratorFromPointCloud(ros::NodeHandle &nh) : PoseGeneratorBase(nh){
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
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

long PoseGeneratorFromPointCloud::getNumberOfPoints(){

    long count = space->points.size();
    count *= (long) (config_.pitch_range / config_.step_pitch);
    count *= (long) (config_.roll_range / config_.step_roll);
    count *= (long) (config_.yaw_range / config_.step_yaw);
    return count;
}

bool PoseGeneratorFromPointCloud::parseConfig(ros::NodeHandle &nh) {

    GET_PARAM_REQUIRED(nh,"roll_default",config_.roll_default);
    GET_PARAM_REQUIRED(nh,"pitch_default",config_.pitch_default);
    GET_PARAM_REQUIRED(nh,"yaw_default",config_.yaw_default);
    GET_PARAM_REQUIRED(nh,"roll_range",config_.roll_range);
    GET_PARAM_REQUIRED(nh,"pitch_range",config_.pitch_range);
    GET_PARAM_REQUIRED(nh,"yaw_range",config_.yaw_range);
    GET_PARAM_REQUIRED(nh,"step_roll",config_.step_roll);
    GET_PARAM_REQUIRED(nh,"step_pitch",config_.step_pitch);
    GET_PARAM_REQUIRED(nh,"step_yaw",config_.step_yaw);
    GET_PARAM_REQUIRED(nh,"mesh",config_.mesh);

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

sensor_msgs::PointCloud2 PoseGeneratorFromPointCloud::getPointCloud2(){
    return point_cloud_msg_;
}
