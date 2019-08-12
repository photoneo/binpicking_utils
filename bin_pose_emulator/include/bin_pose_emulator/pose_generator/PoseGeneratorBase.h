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
#include <sensor_msgs/PointCloud2.h>

class PoseGeneratorBase {
public:
    enum GeneratorType {
        CUBE,
        CUBE_RANDOM,
        POINT_CLOUD,
        POINT_CLOUD_RANDOM,
        SINGLE_POSE
    };

    PoseGeneratorBase(ros::NodeHandle& nh);
    bool getPose(geometry_msgs::Pose& pose, double approachDistance = APPROACH_DIST_DEFAULT);
    virtual bool generate(geometry_msgs::Pose& pose) = 0;
    virtual bool parseConfig(ros::NodeHandle& nh) = 0;
    virtual long getNumberOfPoints() = 0;
    virtual sensor_msgs::PointCloud2 getPointCloud2();
protected:
    virtual void visualizeBin() = 0;
    void visualizePose(const geometry_msgs::Pose& graspPose, double arrowDistance = APPROACH_DIST_DEFAULT, bool multiArray = false);


    tf::Transform broadcastPoseTF(const geometry_msgs::Pose& graspPose);
    tf::TransformBroadcaster broadcaster;
    ros::Publisher markerPub;

private:
    const static int APPROACH_DIST_DEFAULT = 0.3;

};
#endif //PROJECT_POSEGENERATORBASE_H
