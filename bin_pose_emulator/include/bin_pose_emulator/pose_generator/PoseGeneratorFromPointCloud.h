//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMPOINTCLOUD_H
#define PROJECT_POSEGENERATORFROMPOINTCLOUD_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>


class PoseGeneratorFromPointCloud : public PoseGeneratorBase {
public:
    struct ConfigData {
        std::string mesh;

        double rollMin;
        double rollMax;
        double pitchMin;
        double pitchMax;
        double yawMin;
        double yawMax;

        double stepRoll;
        double stepPitch;
        double stepYaw;
    };

    PoseGeneratorFromPointCloud(ros::NodeHandle& nh);
    virtual bool generate(geometry_msgs::Pose& pose);
    virtual bool parseConfig(ros::NodeHandle& nh);
    virtual long getNumberOfPoints();
    virtual sensor_msgs::PointCloud2 getPointCloud2() override;


protected:
    virtual void visualizeBin();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> space;
    ConfigData config;

private:
    ros::Publisher pointCloudPub;
    sensor_msgs::PointCloud2 pointCloudMsg;
};

#endif //PROJECT_POSEGENERATORFROMPOINTCLOUD_H
