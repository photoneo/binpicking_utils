//
// Created by controller on 3/12/19.
//


#include "bin_pose_emulator/ActionServerInterface.h"

#include <pho_localization_msgs/LocalizationObject.h>
#include <pho_localization_msgs/SceneSourceStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <pho_localization_msgs/PointCloud.h>
#include <bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloud.h>
#include <bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloudRandom.h>
#include <bin_pose_emulator/pose_generator/PoseGeneratorFromCube.h>
#include <bin_pose_emulator/pose_generator/PoseGeneratorFromCubeRandom.h>
#include <bin_pose_emulator/pose_generator/SinglePoseGenerator.h>
#include <bin_pose_emulator/BinPoseEmulatorException.h>
#include <bin_pose_emulator/Utils.h>

ActionServerInterface::ActionServerInterface(ros::NodeHandle& nh) :
nh(nh)
{
    int binVolumeType = 0;
    GET_PARAM_REQUIRED(nh,"bin_volume_type",binVolumeType);

    switch (binVolumeType){
        case PoseGeneratorBase::CUBE:
            poseGenerator = std::make_shared<PoseGeneratorFromCube>(nh);
            break;
        case PoseGeneratorBase::CUBE_RANDOM:
            poseGenerator = std::make_shared<PoseGeneratorFromCubeRandom>(nh);
            break;
        case PoseGeneratorBase::POINT_CLOUD:
            poseGenerator = std::make_shared<PoseGeneratorFromPointCloud>(nh);
            break;
        case PoseGeneratorBase::POINT_CLOUD_RANDOM:
            poseGenerator = std::make_shared<PoseGeneratorFromPointCloudRandom>(nh);
            break;
        case PoseGeneratorBase::SINGLE_POSE:
            poseGenerator = std::make_shared<SinglePoseGenerator>(nh);
            break;
         default:
             throw BinPoseEmulatorException("Wrong bin volume type: " + std::to_string(binVolumeType));
    }

    poseGenerator->parseConfig(nh);

    as.reset(new actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction>
            (nh, "scan_and_locate", boost::bind(&ActionServerInterface::actionServerCallback, this, _1), false));

    statusPublisher = nh.advertise<pho_localization_msgs::SceneSourceStatus>("scene_source_status", 100);

    //start ROS interface
    as->start();
}

void ActionServerInterface::actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal) {

    int objectId = 0;
    static int pFrame = 0;
    acquisitionComplete(pFrame);
    ros::Duration(0.2).sleep();
    publishEmptyCloud(pFrame);

    ros::Duration(1.0).sleep();

    long num_of_position = poseGenerator->getNumberOfPoints();
    ROS_INFO("Simulation started. Prepared are %d poses", num_of_position);

    pho_localization::ScanAndLocateFeedback feedback;
    for (int i = 0; i < num_of_position; i++) {
        feedback.object.header.seq = objectId;
        feedback.object.header.frame_id = "base_link";
        feedback.object.header.stamp = ros::Time::now();
        feedback.object.id = objectId++;
        feedback.object.occluded = false;
        feedback.object.p_frame_id = pFrame;
        feedback.object.visibleOverlap = true;
        if (!poseGenerator->getPose(feedback.object.pose, 0.3)) {
            break;
        }
        ROS_DEBUG_STREAM(feedback.object);
        as->publishFeedback(feedback);
        ros::Duration(0.1).sleep();
    }

    pho_localization::ScanAndLocateResult result;
    result.error_code.val = pho_localization_msgs::PhoLocalizationErrorCodes::SUCCESS;
    as->setSucceeded(result);

    pFrame++;
}

void ActionServerInterface::publishEmptyCloud(int frameId) {

    ROS_DEBUG("Point cloud available. Frame id: %d",frameId);
    pho_localization_msgs::SceneSourceStatus status;
    status.sceneSourceStatusType.val = pho_localization_msgs::SceneSourceStatusTypes::POINT_CLOUD_AVAILABLE;

    pho_localization_msgs::PointCloud cloud_msg;
    cloud_msg.pointCloud = poseGenerator->getPointCloud2();
    cloud_msg.scanType.val = 1;
    cloud_msg.pointCloud.header.stamp = ros::Time::now();
    cloud_msg.pointCloud.header.frame_id = "base_link";
    cloud_msg.pointCloud.header.seq = frameId;
    status.pointCloud = cloud_msg;

    statusPublisher.publish(status);
}

void ActionServerInterface::acquisitionComplete(int frameId) {
    ROS_DEBUG("Aquisition complete. Frame id: %d",frameId);
    pho_localization_msgs::SceneSourceStatus status;
    status.sceneSourceStatusType.val = pho_localization_msgs::SceneSourceStatusTypes::ACQUISITION_COMPLETE;
    statusPublisher.publish(status);
}

