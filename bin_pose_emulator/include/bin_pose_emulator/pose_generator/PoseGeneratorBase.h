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
    bool getPose(geometry_msgs::Pose &pose, double approach_distance = APPROACH_DIST_DEFAULT);
    virtual bool generate(geometry_msgs::Pose &pose) = 0;
    virtual bool parseConfig(std::string filepath) = 0;
    virtual long getNumberOfPoints() = 0;
protected:
    virtual void visualizeBin() = 0;
    void visualizePose(geometry_msgs::Pose grasp_pose, double arrow_distance = APPROACH_DIST_DEFAULT, bool multiArray = false);


    tf::Transform broadcastPoseTF(const geometry_msgs::Pose &grasp_pose);
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher marker_pub_;

private:
    const static int APPROACH_DIST_DEFAULT = 0.3;

};
#endif //PROJECT_POSEGENERATORBASE_H
