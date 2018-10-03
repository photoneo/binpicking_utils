//
// Created by controller on 9/26/18.
//

#include "binpicking_emulator/path_length_test.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

PathLengthTest::PathLengthTest(){

    ros::NodeHandle nh;

    fk_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

    save_log_server_ = nh.advertiseService("/binpicking_emulator/save_log", &PathLengthTest::saveLogCallback, this);

    fk_srv_.request.header.frame_id = "base_link";
    fk_srv_.request.fk_link_names = {"tool1"};
    fk_srv_.request.robot_state.joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    joints_.resize(6);
}

bool PathLengthTest::computeFk(const std::vector<double> start_pose_from_robot_, geometry_msgs::Pose &computed_fk) {

    fk_srv_.request.header.stamp = ros::Time::now();
    fk_srv_.request.robot_state.joint_state.position = start_pose_from_robot_;

    if (fk_client_.call(fk_srv_)){
        if (fk_srv_.response.error_code.val == 1){
            computed_fk = fk_srv_.response.pose_stamped[0].pose;
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

double PathLengthTest::getAngle(const geometry_msgs::Quaternion start, const geometry_msgs::Quaternion end) {

    tf::Quaternion tf_start, tf_end;
    tf::quaternionMsgToTF(start, tf_start);
    tf::quaternionMsgToTF(end, tf_end);

    Eigen::Quaterniond eigen_start_quat, eigen_end_quat;
//
//    tf::quaternionTFToEigen(tf_start, eigen_start_quat);
//    tf::quaternionTFToEigen(tf_end, eigen_end_quat);
//
//
//    Eigen::Matrix3d matrix_start = eigen_start_quat.toRotationMatrix();
//    Eigen::Matrix3d matrix_end = eigen_end_quat.toRotationMatrix();
//
//    Eigen::Vector3d vector_ones = Eigen::Vector3d::Ones();
//
//    Eigen::Vector3d vector_start =  matrix_start * vector_ones;
//    Eigen::Vector3d vector_end =  matrix_end * vector_ones;
//
//    double angle_1 = vector_start.dot(vector_end);

    tf::Quaternion q = tf_end * tf_start.inverse();
   double angle = q.getAngle();

    if (angle > M_PI) {
        angle -= M_PI*2;
    }

//    ROS_ERROR("vypocitany uhol 1 %f, uhol 2 %f", angle_1, angle_2);

    return angle;
}

bool PathLengthTest::setStart(const std::vector<double> start_pose_from_robot_) {

    return computeFk(start_pose_from_robot_, start_pose_);
}

void PathLengthTest::addPoint(const geometry_msgs::Pose pose) {

    mutex_.lock();
    forward_distances_.push_back(getDistance(start_pose_.position, pose.position));
    forward_angles_.push_back(getAngle(start_pose_.orientation, pose.orientation));
    mutex_.unlock();
   // sleep(1);
}

bool PathLengthTest::addPath(const moveit::planning_interface::MoveGroupInterface::Plan &plan){

    geometry_msgs::Pose last_pose, new_pose;
    double path_lenth = 0, path_angle = 0;

    last_pose = start_pose_;

    for (auto joint_state : plan.trajectory_.joint_trajectory.points){
        if (computeFk(joint_state.positions, new_pose)) {
            path_lenth += getDistance(last_pose.position, new_pose.position);
            path_angle += getAngle(last_pose.orientation, new_pose.orientation);
            last_pose = new_pose;
        }
    }

    mutex_.lock();
    path_distances_.push_back(path_lenth);
    path_angles_.push_back(path_angle);
    mutex_.unlock();
}

void PathLengthTest::addJoint(int idx, double value){
    joints_[idx].push_back(value);
}

bool PathLengthTest::saveLog()
{
    mutex_.lock();
    std::ofstream outfile_results("/home/controller/catkin_ws/planner_test/trajectory_test.txt");
    for (int i = 0; i < path_distances_.size(); i++){
        outfile_results << i + 1 << ", " << path_distances_[i] << ", " << forward_distances_[i] << ", " << path_angles_[i] << ", " << forward_angles_[i];
        for (int j = 0; j < 6; j++){
            outfile_results << ", " << joints_[j][i];
        }
        outfile_results << "\n";
    }

    outfile_results.close();
    mutex_.unlock();
    if (path_distances_.size() > 0){
        return true;
    } else {
        return false;
    }
}

bool PathLengthTest::saveLogCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = saveLog();
    return true;
}