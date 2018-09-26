//
// Created by controller on 9/26/18.
//

#include "binpicking_emulator/path_length_test.h"

PathLengthTest::PathLengthTest(){

    ros::NodeHandle nh;

    fk_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

    save_log_server_ = nh.advertiseService("/binpicking_emulator/save_log", &PathLengthTest::saveLogCallback, this);

    fk_srv_.request.header.frame_id = "tool1";
    fk_srv_.request.header.stamp = ros::Time::now();
    fk_srv_.request.fk_link_names = {"tool1"};
    fk_srv_.request.robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
}

bool PathLengthTest::computeFk(const std::vector<double> start_pose_from_robot_) {

    fk_srv_.request.robot_state.joint_state.position = start_pose_from_robot_;

    if (fk_client_.call(fk_srv_)){
        if (fk_srv_.response.error_code.val == 1){
            start_point_ = fk_srv_.response.pose_stamped[0].pose.position;
            return true;

        } else{
            return false;
        }

    }else{
        return false;
    }
}

double PathLengthTest::getDistance(const geometry_msgs::Point start, const geometry_msgs::Point end) {

    return sqrt(pow(start.x - end.x, 2.0) +
                pow(start.y - end.y, 2.0) +
                pow(start.z - end.z, 2.0));
}

bool PathLengthTest::setStart(const std::vector<double> start_pose_from_robot_) {

    moveit_msgs::GetPositionFK fk_req;

    fk_req.request.header.frame_id = "tool1";
    fk_req.request.header.stamp = ros::Time::now();
    fk_req.request.fk_link_names = {"tool1"};
    fk_req.request.robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    return computeFk(start_pose_from_robot_);
}

void PathLengthTest::addPoint(const geometry_msgs::Pose pose) {

    mutex_.lock();
    forward_distances_.push_back(getDistance(start_point_, pose.position));
    mutex_.unlock();
}

bool PathLengthTest::addPath(const moveit::planning_interface::MoveGroupInterface::Plan &plan){

    geometry_msgs::Point last_point;
    double path_lenth = 0;

    last_point = start_point_;

    for (int i = 1; i < plan.trajectory_.joint_trajectory.points.size(); i++) {

        if (computeFk(plan.trajectory_.joint_trajectory.points[i].positions)){

            path_lenth += getDistance(last_point, fk_srv_.response.pose_stamped[0].pose.position);
            last_point = fk_srv_.response.pose_stamped[0].pose.position;
        }
    }

    mutex_.lock();
    path_distances_.push_back(path_lenth);
    mutex_.unlock();
}

bool PathLengthTest::saveLogCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    mutex_.lock();
    std::ofstream outfile_results("/home/controller/catkin_ws/planner_test/trajectory_test.txt");
    for (int i = 0; i < path_distances_.size(); i++){
        outfile_results << path_distances_[i] << ", " << forward_distances_[i] << "\n";
    }

    outfile_results.close();
    mutex_.unlock();
    if (path_distances_.size() > 0){
        res.success = true;

    } else {
        res.success = false;
    }
    return true;
}