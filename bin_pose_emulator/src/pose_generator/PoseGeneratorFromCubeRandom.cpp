//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromCubeRandom.h"

PoseGeneratorFromCubeRandom::PoseGeneratorFromCubeRandom(ros::NodeHandle &nh) : PoseGeneratorFromCube(nh){
    srandom(time(NULL));
}

bool PoseGeneratorFromCubeRandom::generate(geometry_msgs::Pose &pose) {

    pose.position.x = randGen( - config_.bin_size_x / 2, config_.bin_size_x / 2);
    pose.position.y = randGen( - config_.bin_size_y / 2, config_.bin_size_y / 2);
    pose.position.z = randGen( - config_.bin_size_z / 2, config_.bin_size_z / 2);

    double roll = randGen(config_.roll_default - config_.roll_range / 2,
                                config_.roll_default + config_.roll_range / 2);
    double pitch = randGen(config_.pitch_default - config_.pitch_range / 2,
                                 config_.pitch_default + config_.pitch_range / 2);
    double yaw = randGen(config_.yaw_default - config_.yaw_range / 2,
                               config_.yaw_default + config_.yaw_range / 2);


    tf::Quaternion grasp_orientation;
    grasp_orientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = grasp_orientation.getX();
    pose.orientation.y = grasp_orientation.getY();
    pose.orientation.z = grasp_orientation.getZ();
    pose.orientation.w = grasp_orientation.getW();

    transformPose(pose);
}

double PoseGeneratorFromCubeRandom::randGen(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
