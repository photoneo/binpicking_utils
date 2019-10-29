//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromCube.h"
#include <visualization_msgs/Marker.h>
#include <bin_pose_emulator/Utils.h>


PoseGeneratorFromCube::PoseGeneratorFromCube(ros::NodeHandle& nh) : PoseGeneratorBase(nh){

    transformBin.setOrigin(tf::Vector3(0,0,0));
    transformBin.setRotation(tf::Quaternion(0,0,0,1));
}

bool PoseGeneratorFromCube::generate(geometry_msgs::Pose& pose){

    static double x =  -config.binSizeX / 2;
    static double y =  -config.binSizeY / 2;
    static double z =  -config.binSizeZ / 2;

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
                x += config.stepX;

                if (x > config.binSizeX / 2) {
                    x = -config.binSizeX / 2;
                    y += config.stepY;

                    if (y > config.binSizeY / 2) {
                        y = -config.binSizeY / 2;
                        z += config.stepZ;

                        if (z > config.binSizeZ / 2) {

                            x = -config.binSizeX / 2;
                            y = -config.binSizeY / 2;
                            z = -config.binSizeZ / 2;

                            roll = config.rollMin;
                            pitch = config.pitchMin;
                            yaw = config.yawMin;
                            return false;
                        }
                    }
                }
            }
        }
    }

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf::Quaternion graspOrientation;
    graspOrientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = graspOrientation.getX();
    pose.orientation.y = graspOrientation.getY();
    pose.orientation.z = graspOrientation.getZ();
    pose.orientation.w = graspOrientation.getW();

    transformPose(pose);
    return true;
}

long PoseGeneratorFromCube::getNumberOfPoints(){

    double rollRange = config.rollMax - config.rollMin;
    double pitchRange = config.pitchMax - config.pitchMin;
    double yawRange = config.yawMax - config.yawMin;

    long count = ((long)(config.binSizeX / config.stepX) + 1);
    count *=  ((long)(config.binSizeY / config.stepY) + 1);
    count *=  ((long)(config.binSizeZ / config.stepZ) + 1);
    count *=  ((long)(rollRange / config.stepRoll) + 1);
    count *=  ((long)(pitchRange / config.stepPitch) + 1);
    count *=  ((long)(yawRange / config.stepYaw) + 1);
    
    return count;
}

bool PoseGeneratorFromCube::parseConfig(ros::NodeHandle& nh) {

    GET_PARAM_REQUIRED(nh,"bin_center_x",config.binCenterX);
    GET_PARAM_REQUIRED(nh,"bin_center_y",config.binCenterY);
    GET_PARAM_REQUIRED(nh,"bin_center_z",config.binCenterZ);
    GET_PARAM_REQUIRED(nh,"bin_size_x",config.binSizeX);
    GET_PARAM_REQUIRED(nh,"bin_size_y",config.binSizeY);
    GET_PARAM_REQUIRED(nh,"bin_size_z",config.binSizeZ);
    GET_PARAM_REQUIRED(nh,"step_x",config.stepX);
    GET_PARAM_REQUIRED(nh,"step_y",config.stepY);
    GET_PARAM_REQUIRED(nh,"step_z",config.stepZ);

    GET_PARAM_REQUIRED(nh,"roll_min",config.rollMin);
    GET_PARAM_REQUIRED(nh,"pitch_min",config.pitchMin);
    GET_PARAM_REQUIRED(nh,"yaw_min",config.yawMin);
    GET_PARAM_REQUIRED(nh,"roll_max",config.rollMax);
    GET_PARAM_REQUIRED(nh,"pitch_max",config.pitchMax);
    GET_PARAM_REQUIRED(nh,"yaw_max",config.yawMax);

    GET_PARAM_REQUIRED(nh,"step_roll",config.stepRoll);
    GET_PARAM_REQUIRED(nh,"step_pitch",config.stepPitch);
    GET_PARAM_REQUIRED(nh,"step_yaw",config.stepYaw);
    GET_PARAM_REQUIRED(nh,"x_rotation",config.xRotation);
    GET_PARAM_REQUIRED(nh,"y_rotation",config.yRotation);
    GET_PARAM_REQUIRED(nh,"z_rotation",config.zRotation);

    transformBin.setOrigin(tf::Vector3(config.binCenterX, config.binCenterY, config.binCenterZ));
    tf::Quaternion q;
    q.setRPY(config.xRotation, config.yRotation,config.zRotation);
    transformBin.setRotation(q);
}

void PoseGeneratorFromCube::visualizeBin()
{
    //Create transformation, set origin and rotation and finally send
    broadcaster.sendTransform(tf::StampedTransform(transformBin, ros::Time::now(),"base_link", "bin"));

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/bin";
    marker.header.stamp = ros::Time::now();

    marker.ns = "bin";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = config.binSizeX;
    marker.scale.y = config.binSizeY;
    marker.scale.z = config.binSizeZ;

    marker.color.r = 0.8f;
    marker.color.g = 0.0f;
    marker.color.b = 0.8f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    markerPub.publish(marker);
}

void PoseGeneratorFromCube::transformPose(geometry_msgs::Pose& pose){
    tf::Transform objectTransform;
    tf::poseMsgToTF(pose, objectTransform);
    tf::Transform result = transformBin * objectTransform;
    tf::poseTFToMsg(result, pose);
}
