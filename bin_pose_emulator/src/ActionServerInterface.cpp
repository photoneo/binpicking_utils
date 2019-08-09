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
#include <bin_pose_emulator/utils.h>

ActionServerInterface::ActionServerInterface(ros::NodeHandle &nh) :
nh_(nh)
{
    int bin_volume_type = 0;
    GET_PARAM_REQUIRED(nh,"bin_volume_type",bin_volume_type);

    ROS_INFO("Loaded bin volume type %d", bin_volume_type);
    switch (bin_volume_type){
        case PoseGeneratorBase::CUBE:
            pose_generator_ = std::make_shared<PoseGeneratorFromCube>(nh_);
            break;
        case PoseGeneratorBase::CUBE_RANDOM:
            pose_generator_ = std::make_shared<PoseGeneratorFromCubeRandom>(nh_);
            break;
        case PoseGeneratorBase::POINT_CLOUD:
            pose_generator_ = std::make_shared<PoseGeneratorFromPointCloud>(nh_);
            break;
        case PoseGeneratorBase::POINT_CLOUD_RANDOM:
            pose_generator_ = std::make_shared<PoseGeneratorFromPointCloudRandom>(nh_);
            break;
        case PoseGeneratorBase::SINGLE_POSE:
            //pose_generator_ = std::make_shared<PoseGeneratorFromPointCloud>(nh_);
           // break;

           default:

                break;
    }

    pose_generator_->parseConfig(nh);

    as_.reset(new actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction>
            (nh_, "scan_and_locate", boost::bind(&ActionServerInterface::actionServerCallback, this, _1), false));

    statusPublisher = nh_.advertise<pho_localization_msgs::SceneSourceStatus>("scene_source_status", 100);

    //start ROS interface
    as_->start();
}

void ActionServerInterface::actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal) {

    int object_id = 0;
    static int p_frame = 0;
    acquisitionComplete(p_frame);
    ros::Duration(0.2).sleep();
    publishEmptyCloud(p_frame);

    ros::Duration(1.0).sleep();

    long num_of_position = pose_generator_->getNumberOfPoints();
    ROS_WARN("Simulation started. Prepared are %d poses", num_of_position);

    pho_localization::ScanAndLocateFeedback feedback;
    for (int i = 0; i < num_of_position; i++) {
        feedback.object.header.seq = object_id;
        feedback.object.header.frame_id = "base_link";
        feedback.object.header.stamp = ros::Time::now();
        feedback.object.id = object_id++;
        feedback.object.occluded = false;
        feedback.object.p_frame_id = p_frame;
        feedback.object.visibleOverlap = true;
        pose_generator_->getPose(feedback.object.pose, 0.3);
        ROS_INFO_STREAM(feedback.object);
        as_->publishFeedback(feedback);
        ros::Duration(0.1).sleep();
    }

    pho_localization::ScanAndLocateResult result;
    result.error_code.val = pho_localization_msgs::PhoLocalizationErrorCodes::SUCCESS;
    as_->setSucceeded(result);

    p_frame++;
}


void ActionServerInterface::publishEmptyCloud(int frameId) {

    ROS_DEBUG("Point cloud available. Frame id: %d",frameId);
    pho_localization_msgs::SceneSourceStatus status;
    status.sceneSourceStatusType.val = pho_localization_msgs::SceneSourceStatusTypes::POINT_CLOUD_AVAILABLE;

    pho_localization_msgs::PointCloud cloud_msg;
    cloud_msg.pointCloud = pose_generator_->getPointCloud2();

  //  geometry_msgs::Transform tr(1.823,-0.365,1.499, 1);
   // cloud_msg.sensorOrigin = Eigen::Vector4f(1.823,-0.365,1.499, 1);

    cloud_msg.scanType.val = 1;

 //   pcl::toROSMsg(cloud, cloudMsg);
   // std::cout << cloudMsg.header.frame_id << std::endl;
    cloud_msg.pointCloud.header.stamp = ros::Time::now();
    cloud_msg.pointCloud.header.frame_id = "base_link";
   // std::cout << "header seq " << headerSeq << std::endl;
    cloud_msg.pointCloud.header.seq = frameId;
   // std::cout << cloud_msg.header.seq << std::endl;
    status.pointCloud = cloud_msg;

    statusPublisher.publish(status);
}

void ActionServerInterface::acquisitionComplete(int frameId){
    ROS_DEBUG("Aquisition complete. Frame id: %d",frameId);
    pho_localization_msgs::SceneSourceStatus status;
    status.sceneSourceStatusType.val = pho_localization_msgs::SceneSourceStatusTypes::ACQUISITION_COMPLETE;
    statusPublisher.publish(status);
}

