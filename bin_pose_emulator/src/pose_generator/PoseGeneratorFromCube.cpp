//
// Created by controller on 7/10/19.
//


#include "bin_pose_emulator/pose_generator/PoseGeneratorFromCube.h"

#include <yaml-cpp/yaml.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <bin_pose_emulator/utils.h>


PoseGeneratorFromCube::PoseGeneratorFromCube(ros::NodeHandle &nh) : PoseGeneratorBase(nh){

    transform_bin_.setOrigin(tf::Vector3(0,0,0));
    transform_bin_.setRotation(tf::Quaternion(0,0,0,1));
}

bool PoseGeneratorFromCube::generate(geometry_msgs::Pose &pose){

    static double x =  -config_.bin_size_x / 2;
    static double y =  -config_.bin_size_y / 2;
    static double z =  -config_.bin_size_z / 2;

    static double roll = config_.roll_default -config_.roll_range / 2;
    static double pitch = config_.pitch_default -config_.pitch_range / 2;
    static double yaw = config_.yaw_default -config_.yaw_range / 2;

    yaw += config_.step_yaw;
    if (yaw >= config_.yaw_default + config_.yaw_range / 2){
        yaw = config_.yaw_default - config_.yaw_range / 2;
        pitch += config_.step_pitch;

        if (pitch >= config_.pitch_default + config_.pitch_range / 2) {
            pitch = config_.pitch_default - config_.pitch_range / 2;
            roll += config_.step_roll;

            if (roll >= config_.roll_default + config_.roll_range / 2) {
                roll = config_.roll_default - config_.roll_range / 2;
                x += config_.step_x;

                if (x >= config_.bin_size_x / 2) {
                    x = -config_.bin_size_x / 2;
                    y += config_.step_y;

                    if (y >= config_.bin_size_y / 2) {
                        y = -config_.bin_size_y / 2;
                        z += config_.step_z;

                        if (z >= config_.bin_size_z / 2) {

                            x = -config_.bin_size_x / 2;
                            y = -config_.bin_size_y / 2;
                            z = -config_.bin_size_z / 2;

                            roll = config_.roll_default -config_.roll_range / 2;
                            pitch = config_.pitch_default -config_.pitch_range / 2;
                            yaw = config_.yaw_default -config_.yaw_range / 2;
                            return true;
                        }
                    }
                }
            }
        }
    }

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf::Quaternion grasp_orientation;
    grasp_orientation.setRPY(roll, pitch, yaw);
    pose.orientation.x = grasp_orientation.getX();
    pose.orientation.y = grasp_orientation.getY();
    pose.orientation.z = grasp_orientation.getZ();
    pose.orientation.w = grasp_orientation.getW();

    transformPose(pose);
}

long PoseGeneratorFromCube::getNumberOfPoints(){

    long count = (long)(config_.bin_size_x / config_.step_x);
    count *=  (long)(config_.bin_size_y / config_.step_y);
    count *=  (long)(config_.bin_size_z / config_.step_z);
    count *=  (long)(config_.pitch_range / config_.step_pitch);
    count *=  (long)(config_.roll_range / config_.step_roll);
    count *=  (long)(config_.yaw_range / config_.step_yaw);

    return count;
}

bool PoseGeneratorFromCube::parseConfig(ros::NodeHandle &nh) {

    GET_PARAM_REQUIRED(nh,"bin_center_x",config_.bin_center_x);
    GET_PARAM_REQUIRED(nh,"bin_center_y",config_.bin_center_y);
    GET_PARAM_REQUIRED(nh,"bin_center_z",config_.bin_center_z);
    GET_PARAM_REQUIRED(nh,"bin_size_x",config_.bin_size_x);
    GET_PARAM_REQUIRED(nh,"bin_size_y",config_.bin_size_y);
    GET_PARAM_REQUIRED(nh,"bin_size_z",config_.bin_size_z);
    GET_PARAM_REQUIRED(nh,"step_x",config_.step_x);
    GET_PARAM_REQUIRED(nh,"step_y",config_.step_y);
    GET_PARAM_REQUIRED(nh,"step_z",config_.step_z);
    GET_PARAM_REQUIRED(nh,"roll_default",config_.roll_default);
    GET_PARAM_REQUIRED(nh,"pitch_default",config_.pitch_default);
    GET_PARAM_REQUIRED(nh,"yaw_default",config_.yaw_default);
    GET_PARAM_REQUIRED(nh,"roll_range",config_.roll_range);
    GET_PARAM_REQUIRED(nh,"pitch_range",config_.pitch_range);
    GET_PARAM_REQUIRED(nh,"yaw_range",config_.yaw_range);
    GET_PARAM_REQUIRED(nh,"step_roll",config_.step_roll);
    GET_PARAM_REQUIRED(nh,"step_pitch",config_.step_pitch);
    GET_PARAM_REQUIRED(nh,"step_yaw",config_.step_yaw);
    GET_PARAM_REQUIRED(nh,"x_rotation",config_.x_rotation);
    GET_PARAM_REQUIRED(nh,"y_rotation",config_.y_rotation);
    GET_PARAM_REQUIRED(nh,"z_rotation",config_.z_rotation);

    transform_bin_.setOrigin(tf::Vector3(config_.bin_center_x, config_.bin_center_y, config_.bin_center_z));
    tf::Quaternion q;
    q.setRPY(config_.x_rotation, config_.y_rotation,config_.z_rotation);
    transform_bin_.setRotation(q);

}

void PoseGeneratorFromCube::visualizeBin()
{
    //Create transformation, set origin and rotation and finally send
    broadcaster_.sendTransform(tf::StampedTransform(transform_bin_, ros::Time::now(),"base_link", "bin"));

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/bin";
    marker.header.stamp = ros::Time::now();

    marker.ns = "bin";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = config_.bin_size_x;
    marker.scale.y = config_.bin_size_y;
    marker.scale.z = config_.bin_size_z;

    marker.color.r = 0.8f;
    marker.color.g = 0.0f;
    marker.color.b = 0.8f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    marker_pub_.publish(marker);
}

void PoseGeneratorFromCube::transformPose(geometry_msgs::Pose &pose){
    tf::Transform object_transform;
    tf::poseMsgToTF(pose, object_transform);
    tf::Transform result = transform_bin_ * object_transform;
    tf::poseTFToMsg(result, pose);
}
