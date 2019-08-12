//
// Created by controller on 8/12/19.
//

#ifndef PROJECT_SINGLEPOSE_H
#define PROJECT_SINGLEPOSE_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>


class SinglePoseGenerator : public PoseGeneratorBase {
public:

    SinglePoseGenerator(ros::NodeHandle& nh);
    virtual bool generate(geometry_msgs::Pose& pose);
    virtual bool parseConfig(ros::NodeHandle& nh);
    virtual long getNumberOfPoints();

protected:
    virtual void visualizeBin();

private:
    geometry_msgs::Pose pose;
    ros::Publisher pointCloudPub;
    sensor_msgs::PointCloud2 pointCloudMsg;
};

#endif //PROJECT_SINGLEPOSE_H
