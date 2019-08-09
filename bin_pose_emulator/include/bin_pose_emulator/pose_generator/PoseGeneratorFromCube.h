//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMCUBE_H
#define PROJECT_POSEGENERATORFROMCUBE_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"

class PoseGeneratorFromCube : public PoseGeneratorBase{
public:
    struct ConfigData {
        // Virtual Bin center
        double bin_center_x;
        double bin_center_y;
        double bin_center_z;

        // Virtual Bin size
        double bin_size_x;
        double bin_size_y;
        double bin_size_z;

        // Virtual Bin rotation around world Z axis
        double x_rotation;
        double y_rotation;
        double z_rotation;

        // Default tool point orientation
        double roll_default;
        double pitch_default;
        double yaw_default;

        // Allowed orientation range
        double roll_range;
        double pitch_range;
        double yaw_range;

        double step_x;
        double step_y;
        double step_z;
        double step_roll;
        double step_pitch;
        double step_yaw;
    };

    PoseGeneratorFromCube(ros::NodeHandle &nh);
    virtual bool generate(geometry_msgs::Pose &pose);
    virtual bool parseConfig(ros::NodeHandle &nh);
    virtual long getNumberOfPoints();



protected:
    virtual void visualizeBin();
    void transformPose(geometry_msgs::Pose &pose);
    ConfigData config_;
    tf::Transform transform_bin_;

private:


};

#endif //PROJECT_POSEGENERATORFROMCUBE_H
