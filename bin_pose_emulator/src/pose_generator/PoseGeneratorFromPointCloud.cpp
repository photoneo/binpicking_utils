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

    static double roll = config.rollDefault -config.rollRange / 2;
    static double pitch = config.pitchDefault -config.pitchRange / 2;
    static double yaw = config.yawDefault -config.yawRange / 2;

    yaw += config.stepYaw;
    if (yaw >= config.yawDefault + config.yawRange / 2) {
        yaw = config.yawDefault - config.yawRange / 2;
        pitch += config.stepPitch;

        if (pitch >= config.pitchDefault + config.pitchRange / 2) {
            pitch = config.pitchDefault - config.pitchRange / 2;
            roll += config.stepRoll;

            if (roll >= config.rollDefault + config.rollRange / 2) {
                roll = config.rollDefault - config.rollRange / 2;
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
    count *= (long) (config.pitchRange / config.stepPitch);
    count *= (long) (config.rollRange / config.stepRoll);
    count *= (long) (config.yawRange / config.stepYaw);
    return count;
}

bool PoseGeneratorFromPointCloud::parseConfig(ros::NodeHandle& nh) {

    GET_PARAM_REQUIRED(nh,"roll_default",config.rollDefault);
    GET_PARAM_REQUIRED(nh,"pitch_default",config.pitchDefault);
    GET_PARAM_REQUIRED(nh,"yaw_default",config.yawDefault);
    GET_PARAM_REQUIRED(nh,"roll_range",config.rollRange);
    GET_PARAM_REQUIRED(nh,"pitch_range",config.pitchRange);
    GET_PARAM_REQUIRED(nh,"yaw_range",config.yawRange);
    GET_PARAM_REQUIRED(nh,"step_roll",config.stepRoll);
    GET_PARAM_REQUIRED(nh,"step_pitch",config.stepPitch);
    GET_PARAM_REQUIRED(nh,"step_yaw",config.stepYaw);
    GET_PARAM_REQUIRED(nh,"mesh",config.mesh);

    // Load Ply picked object into point cloud
    space = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());

    pcl::PLYReader plyReader;
    plyReader.read(config.mesh, *space);
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
