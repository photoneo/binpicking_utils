//
// Created by controller on 7/10/19.
//

#ifndef PROJECT_POSEGENERATORFROMCUBERANDOM_H
#define PROJECT_POSEGENERATORFROMCUBERANDOM_H

#include "bin_pose_emulator/pose_generator/PoseGeneratorFromCube.h"

class PoseGeneratorFromCubeRandom : public PoseGeneratorFromCube {
public:
    PoseGeneratorFromCubeRandom(ros::NodeHandle &nh);
    virtual bool generate(geometry_msgs::Pose &pose) override;
protected:
private:
    double randGen(double fMin, double fMax);



};

#endif //PROJECT_POSEGENERATORFROMCUBERANDOM_H
