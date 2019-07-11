//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORBASE_H
#define PROJECT_POSEGENERATORBASE_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/publisher.h>


class PoseGeneratorBase{
public:
    PoseGeneratorBase(ros::NodeHandle &nh);
    bool getPose(geometry_msgs::Pose &pose);
    virtual bool generate(geometry_msgs::Pose &pose) = 0;
    virtual bool parseConfig(std::string filepath) = 0;

protected:
    virtual void visualizeBin() = 0;
    void visualizePose(geometry_msgs::Pose grasp_pose, double arrow_distance = 0.3, bool multiArray = false);


    tf::Transform broadcastPoseTF(const geometry_msgs::Pose &grasp_pose);
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher marker_pub_;

private:

};
#endif //PROJECT_POSEGENERATORBASE_H
