//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromCubeRandom.h"

PoseGeneratorFromCubeRandom::PoseGeneratorFromCubeRandom(ros::NodeHandle& nh) : PoseGeneratorFromCube(nh){
    srandom(time(NULL));
}

bool PoseGeneratorFromCubeRandom::generate(geometry_msgs::Pose& pose) {

    pose.position.x = randGen( - config.binSizeX / 2, config.binSizeX / 2);
    pose.position.y = randGen( - config.binSizeY / 2, config.binSizeY / 2);
    pose.position.z = randGen( - config.binSizeZ / 2, config.binSizeZ / 2);

    double roll = randGen(config.rollMin, config.rollMax);
    double pitch = randGen(config.pitchMin, config.pitchMax);
    double yaw = randGen(config.yawMin, config.yawMax);

    tf::Quaternion graspOrientation;
    graspOrientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = graspOrientation.getX();
    pose.orientation.y = graspOrientation.getY();
    pose.orientation.z = graspOrientation.getZ();
    pose.orientation.w = graspOrientation.getW();

    transformPose(pose);
    return true;
}

double PoseGeneratorFromCubeRandom::randGen(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

long PoseGeneratorFromCubeRandom::getNumberOfPoints(){
    return NUM_OF_POINTS;
}