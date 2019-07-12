//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMPOINTCLOUD_H
#define PROJECT_POSEGENERATORFROMPOINTCLOUD_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace pose_generator {
    struct ConfigData {
        std::string mesh;

        // Default tool point orientation
        double roll_default;
        double pitch_default;
        double yaw_default;

        // Allowed orientation range
        double roll_range;
        double pitch_range;
        double yaw_range;

        double step_roll;
        double step_pitch;
        double step_yaw;
    };
}

class PoseGeneratorFromPointCloud : public PoseGeneratorBase{
public:
    PoseGeneratorFromPointCloud(ros::NodeHandle &nh);
    virtual bool generate(geometry_msgs::Pose &pose);
    virtual bool parseConfig(std::string filepath);


protected:
    virtual void visualizeBin();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> space;
    pose_generator::ConfigData config_;

private:
    ros::Publisher point_cloud_pub_;
    sensor_msgs::PointCloud2 point_cloud_msg_;
};

#endif //PROJECT_POSEGENERATORFROMPOINTCLOUD_H
