//
// Created by controller on 7/10/19.
//

#include "bin_pose_emulator/pose_generator/SinglePoseGenerator.h"

#include <bin_pose_emulator/Utils.h>

SinglePoseGenerator::SinglePoseGenerator(ros::NodeHandle& nh) : PoseGeneratorBase(nh) {
}

bool SinglePoseGenerator::generate(geometry_msgs::Pose& pose) {

    pose = this->pose;
    return true;
}

long SinglePoseGenerator::getNumberOfPoints() {

    return 1;
}

bool SinglePoseGenerator::parseConfig(ros::NodeHandle& nh) {

    GET_PARAM_REQUIRED(nh,"position/x",pose.position.x);
    GET_PARAM_REQUIRED(nh,"position/y",pose.position.y);
    GET_PARAM_REQUIRED(nh,"position/z",pose.position.z);
    GET_PARAM_REQUIRED(nh,"orientation/x",pose.orientation.x);
    GET_PARAM_REQUIRED(nh,"orientation/y",pose.orientation.y);
    GET_PARAM_REQUIRED(nh,"orientation/z",pose.orientation.z);
    GET_PARAM_REQUIRED(nh,"orientation/w",pose.orientation.w);

    return true;
}

void SinglePoseGenerator::visualizeBin()
{

}