//
// Created by controller on 9/26/18.
//

#ifndef PROJECT_PATH_LENGTH_TEST_H
#define PROJECT_PATH_LENGTH_TEST_H

#include <ros/ros.h>

#include <moveit_msgs/GetPositionFK.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <mutex>

class PathLengthTest{
public:
    PathLengthTest();
    bool setStart(const std::vector<double> start_pose_from_robot_);
    void addPoint(const geometry_msgs::Pose pose);
    bool addPath(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

private:
    ros::ServiceServer save_log_server_;
    ros::ServiceClient fk_client_;
    geometry_msgs::Point start_point_;
    moveit_msgs::GetPositionFK fk_srv_;

    bool computeFk(const std::vector<double> start_pose_from_robot_, geometry_msgs::Point &computed_fk);
    double getDistance(const geometry_msgs::Point start, const geometry_msgs::Point end);

    bool saveLogCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    std::vector<double> forward_distances_;
    std::vector<double> path_distances_;
    std::mutex mutex_;
};
#endif //PROJECT_PATH_LENGTH_TEST_H
