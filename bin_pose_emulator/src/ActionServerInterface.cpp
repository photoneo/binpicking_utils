//
// Created by controller on 3/12/19.
//


#include "bin_pose_emulator/ActionServerInterface.h"

#include <pho_localization_msgs/LocalizationObject.h>
#include <sensor_msgs/PointCloud2.h>
#include <pho_localization_msgs/PointCloud.h>
#include <bin_pose_emulator/pose_generator/PoseGeneratorFromCubeRandom.h>

ActionServerInterface::ActionServerInterface( std::string filepath) :
nh_("vision_system_1")
{
    pose_generator_ = std::make_shared<PoseGeneratorFromCubeRandom>(nh_);
    pose_generator_->parseConfig(filepath);

    as_.reset(new actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction>
            (nh_, "scan_and_locate", boost::bind(&ActionServerInterface::actionServerCallback, this, _1), false));

    //start ROS interface
    as_->start();

    ros::AdvertiseOptions ops;
    ops.template init<pho_localization_msgs::PointCloud>("pointcloud", 1);
    ops.latch = false;
    ops.has_header = false;
    cloud_publisher_ = nh_.advertise(ops);
}

void ActionServerInterface::actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal) {

    int object_id = 0;
    static int p_frame = 0;
    publishEmptyCloud(p_frame);

    sleep(2);
    pho_localization::ScanAndLocateFeedback feedback;
    for (int i = 0; i < 10; i++) {
        feedback.object.header.seq = object_id;
        feedback.object.header.frame_id = "base_link";
        feedback.object.header.stamp = ros::Time::now();
        feedback.object.id = object_id++;
        feedback.object.occluded = false;
        feedback.object.p_frame_id = p_frame;
        feedback.object.visibleOverlap = true;
        pose_generator_->getPose(feedback.object.pose);
        ROS_INFO_STREAM(feedback.object);
        as_->publishFeedback(feedback);
        ros::Duration(0.5).sleep();
    }

    pho_localization::ScanAndLocateResult result;
    result.error_code.val = pho_localization_msgs::PhoLocalizationErrorCodes::SUCCESS;
    as_->setSucceeded(result);

    p_frame++;
}


void ActionServerInterface::publishEmptyCloud(uint32_t header_seq) {

    pho_localization_msgs::PointCloud cloud_msg;
  //  geometry_msgs::Transform tr(1.823,-0.365,1.499, 1);
   // cloud_msg.sensorOrigin = Eigen::Vector4f(1.823,-0.365,1.499, 1);

    cloud_msg.scanType.val = 1;

 //   pcl::toROSMsg(cloud, cloudMsg);
   // std::cout << cloudMsg.header.frame_id << std::endl;
    cloud_msg.pointCloud.header.stamp = ros::Time::now();
    cloud_msg.pointCloud.header.frame_id = "base_link";
   // std::cout << "header seq " << headerSeq << std::endl;
    cloud_msg.pointCloud.header.seq = header_seq;
   // std::cout << cloud_msg.header.seq << std::endl;
    cloud_publisher_.publish(cloud_msg);
   // std::cout << cloud_msg.header.seq << std::endl;
}