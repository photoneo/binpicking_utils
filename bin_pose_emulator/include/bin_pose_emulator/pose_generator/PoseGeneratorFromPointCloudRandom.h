//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMPOINTCLOUDRANDOM_H
#define PROJECT_POSEGENERATORFROMPOINTCLOUDRANDOM_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorFromPointCloud.h"

class PoseGeneratorFromPointCloudRandom : public PoseGeneratorFromPointCloud {
public:
    PoseGeneratorFromPointCloudRandom(ros::NodeHandle &nh);
    virtual bool generate(geometry_msgs::Pose &pose) override;
protected:
private:
    double randGen(double fMin, double fMax);



};

#endif //PROJECT_POSEGENERATORFROMPOINTCLOUDRANDOM_H