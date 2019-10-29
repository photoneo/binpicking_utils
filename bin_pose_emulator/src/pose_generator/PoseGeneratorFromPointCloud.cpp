//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloud.h"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bin_pose_emulator/Utils.h>

#include <bin_pose_emulator/BinPoseEmulatorException.h>

PoseGeneratorFromPointCloud::PoseGeneratorFromPointCloud(ros::NodeHandle& nh) : PoseGeneratorBase(nh){
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
}

bool PoseGeneratorFromPointCloud::generate(geometry_msgs::Pose& pose){

    static int iterator = 0;

    static double roll = config.rollMin;
    static double pitch = config.pitchMin;
    static double yaw = config.yawMin;

    yaw += config.stepYaw;
    if (yaw > config.yawMax){
        yaw = config.yawMin;
        pitch += config.stepPitch;

        if (pitch > config.pitchMax) {
            pitch = config.pitchMin;
            roll += config.stepRoll;

            if (roll > config.rollMax) {
                roll = config.rollMin;
                iterator++;
            }
        }
    }

    if (iterator >= space->points.size())
        return false;

    pose.position.x = space->points[iterator].x;
    pose.position.y = space->points[iterator].y;
    pose.position.z = space->points[iterator].z;
    tf::Quaternion graspOrientation;
    graspOrientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = graspOrientation.getX();
    pose.orientation.y = graspOrientation.getY();
    pose.orientation.z = graspOrientation.getZ();
    pose.orientation.w = graspOrientation.getW();
    return true;
}

long PoseGeneratorFromPointCloud::getNumberOfPoints(){

    long count = space->points.size();
    double rollRange = config.rollMax - config.rollMin;
    double pitchRange = config.pitchMax - config.pitchMin;
    double yawRange = config.yawMax - config.yawMin;

    count *=  ((long)(rollRange / config.stepRoll) + 1);
    count *=  ((long)(pitchRange / config.stepPitch) + 1);
    count *=  ((long)(yawRange / config.stepYaw) + 1);

    return count;
}

bool PoseGeneratorFromPointCloud::parseConfig(ros::NodeHandle& nh) {

    GET_PARAM_REQUIRED(nh,"roll_min",config.rollMin);
    GET_PARAM_REQUIRED(nh,"pitch_min",config.pitchMin);
    GET_PARAM_REQUIRED(nh,"yaw_min",config.yawMin);
    GET_PARAM_REQUIRED(nh,"roll_max",config.rollMax);
    GET_PARAM_REQUIRED(nh,"pitch_max",config.pitchMax);
    GET_PARAM_REQUIRED(nh,"yaw_max",config.yawMax);

    GET_PARAM_REQUIRED(nh,"step_roll",config.stepRoll);
    GET_PARAM_REQUIRED(nh,"step_pitch",config.stepPitch);
    GET_PARAM_REQUIRED(nh,"step_yaw",config.stepYaw);
    GET_PARAM_REQUIRED(nh,"mesh",config.mesh);

    // Load Ply picked object into point cloud
    space = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());

    pcl::PLYReader plyReader;
    if (plyReader.read(config.mesh, *space) < 0){
        throw ResourceNotFound(config.mesh);
    }

    if (!space) {
        throw ResourceNotFound(config.mesh);
    }

    for (auto &point : *space){
        point.x /= 1000;
        point.y /= 1000;
        point.z /= 1000;
    }

    pcl::toROSMsg(*space.get(),pointCloudMsg );
    pointCloudMsg.header.frame_id = "base_link";

    return true;
}

void PoseGeneratorFromPointCloud::visualizeBin()
{
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudPub.publish(pointCloudMsg);
}

sensor_msgs::PointCloud2 PoseGeneratorFromPointCloud::getPointCloud2(){
    return pointCloudMsg;
}
