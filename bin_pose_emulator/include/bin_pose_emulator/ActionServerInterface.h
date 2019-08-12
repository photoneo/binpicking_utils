//
// Created by controller on 3/12/19.
//

#ifndef PROJECT_ACTIONSERVER_H
#define PROJECT_ACTIONSERVER_H


//action server
#include <actionlib/server/simple_action_server.h>

//action
#include <pho_localization/ScanAndLocateAction.h>

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"

class ActionServerInterface {

public:
    ActionServerInterface(ros::NodeHandle& nh);

    void actionServerCallback(const pho_localization::ScanAndLocateGoalConstPtr& goal);
protected:
    std::shared_ptr<PoseGeneratorBase> poseGenerator;

private:

    std::shared_ptr<actionlib::SimpleActionServer<pho_localization::ScanAndLocateAction> > as;
    ros::Publisher statusPublisher;
    ros::NodeHandle nh;

    void publishEmptyCloud(int frameId);
    void acquisitionComplete(int frameId);
};

#endif //PROJECT_ACTIONSERVER_H
