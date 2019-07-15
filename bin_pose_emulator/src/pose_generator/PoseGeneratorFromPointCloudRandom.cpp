//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloudRandom.h"

PoseGeneratorFromPointCloudRandom::PoseGeneratorFromPointCloudRandom(ros::NodeHandle &nh) : PoseGeneratorFromPointCloud(nh){
    srandom(time(NULL));
}

bool PoseGeneratorFromPointCloudRandom::generate(geometry_msgs::Pose &pose) {

    int index = randGen(0,  space->points.size()-1);
    pose.position.x = space->points[index].x;
    pose.position.y = space->points[index].y;
    pose.position.z = space->points[index].z;

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
}

double PoseGeneratorFromPointCloudRandom::randGen(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
