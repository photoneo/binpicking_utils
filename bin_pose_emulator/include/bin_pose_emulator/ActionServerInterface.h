//
// Created by controller on 3/12/19.
//

#ifndef PROJECT_ACTIONSERVER_H
#define PROJECT_ACTIONSERVER_H


//action server
#include <actionlib/server/simple_action_server.h>

//action
#include <pho_localization/ScanAndLocateAction.h>

#include "bin_pose_emulator/bin_pose_emulator.h"

class ActionServerInterface{

public:
    ActionServerInterface( std::shared_ptr<BinPoseEmulator> emulator);

    void actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal);

private:

    std::shared_ptr<actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction> > as_;
    ros::Publisher cloud_publisher_;
    ros::NodeHandle nh_;

    std::shared_ptr<BinPoseEmulator> emulator_;

    void publishEmptyCloud(uint32_t header_seq);
};

#endif //PROJECT_ACTIONSERVER_H
