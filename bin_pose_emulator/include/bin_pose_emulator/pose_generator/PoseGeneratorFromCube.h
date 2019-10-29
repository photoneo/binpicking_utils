//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMCUBE_H
#define PROJECT_POSEGENERATORFROMCUBE_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorBase.h"

class PoseGeneratorFromCube : public PoseGeneratorBase {
public:
    struct ConfigData {
        // Virtual Bin center
        double binCenterX;
        double binCenterY;
        double binCenterZ;

        // Virtual Bin size
        double binSizeX;
        double binSizeY;
        double binSizeZ;

        // Virtual Bin rotation around world Z axis
        double xRotation;
        double yRotation;
        double zRotation;

        double rollMin;
        double rollMax;
        double pitchMin;
        double pitchMax;
        double yawMin;
        double yawMax;

        double stepX;
        double stepY;
        double stepZ;
        double stepRoll;
        double stepPitch;
        double stepYaw;
    };

    PoseGeneratorFromCube(ros::NodeHandle& nh);
    virtual bool generate(geometry_msgs::Pose& pose);
    virtual bool parseConfig(ros::NodeHandle& nh);
    virtual long getNumberOfPoints();



protected:
    virtual void visualizeBin();
    void transformPose(geometry_msgs::Pose& pose);
    ConfigData config;
    tf::Transform transformBin;

private:


};

#endif //PROJECT_POSEGENERATORFROMCUBE_H
