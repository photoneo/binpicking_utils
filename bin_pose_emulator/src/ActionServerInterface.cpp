//
// Created by controller on 3/12/19.
//


#include "bin_pose_emulator/ActionServerInterface.h"

#include <pho_localization_msgs/LocalizationObject.h>
#include <sensor_msgs/PointCloud2.h>



ActionServerInterface::ActionServerInterface( std::shared_ptr<BinPoseEmulator> emulator) :
nh_("vision_system_1"),
emulator_(emulator)
{

  //  emulator_ = emulator;

    as_.reset(new actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction>
            (nh_, "scan_and_locate", boost::bind(&ActionServerInterface::actionServerCallback, this, _1), false));

    //start ROS interface
    as_->start();

    ros::AdvertiseOptions ops;
    ops.template init<sensor_msgs::PointCloud2>("point_cloud", 1);
    ops.latch = false;
    ops.has_header = false;
    cloud_publisher_ = nh_.advertise(ops);
}

void ActionServerInterface::actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal) {

    static int object_id = 0;
    //std::cout << "Goal id " << goal->id << std::endl;
    //publishEmptyCloud(goal->id);

    sleep(2);
    pho_localization::ScanAndLocateFeedback feedback;
    for (int i = 0; i < 10; i++) {
        feedback.object.header.seq = 0;
        feedback.object.header.frame_id = "base_link";
        feedback.object.id = object_id;
        feedback.object.occluded = false;
        feedback.object.p_frame_id = object_id++;
        feedback.object.visibleOverlap = true;
        emulator_->getPose(feedback.object.pose);
        ROS_INFO_STREAM(feedback.object);
        as_->publishFeedback(feedback);
        ros::Duration(0.5).sleep();
    }

    pho_localization::ScanAndLocateResult result;
    result.error_code.val = pho_localization_msgs::PhoLocalizationErrorCodes::SUCCESS;
    as_->setSucceeded(result);

}


void ActionServerInterface::publishEmptyCloud(uint32_t header_seq) {

    sensor_msgs::PointCloud2 cloud_msg;
 //   pcl::toROSMsg(cloud, cloudMsg);
   // std::cout << cloudMsg.header.frame_id << std::endl;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "base_link";
   // std::cout << "header seq " << headerSeq << std::endl;
    cloud_msg.header.seq = header_seq;
   // std::cout << cloud_msg.header.seq << std::endl;
    cloud_publisher_.publish(cloud_msg);
   // std::cout << cloud_msg.header.seq << std::endl;
}