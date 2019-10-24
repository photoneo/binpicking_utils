//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloudRandom.h"

PoseGeneratorFromPointCloudRandom::PoseGeneratorFromPointCloudRandom(ros::NodeHandle& nh) : PoseGeneratorFromPointCloud(nh){
    srandom(time(NULL));
}

bool PoseGeneratorFromPointCloudRandom::generate(geometry_msgs::Pose& pose) {

    int index = randGen(0,  space->points.size()-1);
    pose.position.x = space->points[index].x;
    pose.position.y = space->points[index].y;
    pose.position.z = space->points[index].z;

    double roll = randGen(config.rollMin, config.rollMax);
    double pitch = randGen(config.pitchMin, config.pitchMax);
    double yaw = randGen(config.yawMin, config.yawMax);

    tf::Quaternion grasp_orientation;
    grasp_orientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = grasp_orientation.getX();
    pose.orientation.y = grasp_orientation.getY();
    pose.orientation.z = grasp_orientation.getZ();
    pose.orientation.w = grasp_orientation.getW();
    return true;
}

double PoseGeneratorFromPointCloudRandom::randGen(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

long PoseGeneratorFromPointCloudRandom::getNumberOfPoints(){
    return NUM_OF_POINTS;
}