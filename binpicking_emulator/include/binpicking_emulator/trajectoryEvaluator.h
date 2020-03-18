//
// Created by martin on 21.10.2019.
//

#ifndef PROJECT_TRAJECTORYEVALUATOR_H
#define PROJECT_TRAJECTORYEVALUATOR_H

#include <ros/ros.h>
#include <moveit_msgs/MoveGroupResult.h>
#include <moveit_msgs/RobotTrajectory.h>


struct inputTrajectory {
    trajectory_msgs::JointTrajectory jointPositions;
    std::vector <geometry_msgs::Pose> pose;
};

struct eveluatorResult {

    double jointDistance; // how much all joints move
    double cartesianDistance; //how much the tip travel
    double jointJerk;         //how much jerk the joints produce
    double cartesianJerk;     //how much jerk the tip produces
    double jointMaxJerk;      // biggest joint jerk in trajectory
    double cartesianMaxJerk;  //biggest tip jerk in trajectory

    double controlPseudoCost;  //pseudo control cost of whole trajectory(only joint movements makes sense)
    double PoseOrientationDistance; //how much the tip orientation change from ideal movement allong trajectory
};

std::ostream& operator<<(std::ostream& os, const eveluatorResult &e);


class TrajectoryEvaluator {

public:
    enum criterion {
        joinDistance, cartesianDistance, jointJerk, cartesianJerk,
        jointMaxJerk, cartesianMaxJerk, controlPseudoCost, PoseOrientationChange,
        AllCriteria, JointCriteria, CartesianCriteria
    };

    TrajectoryEvaluator() {
        pseudoControlCostValue.push_back(0.7);
        pseudoControlCostValue.push_back(1);
        pseudoControlCostValue.push_back(0.5);
        pseudoControlCostValue.push_back(0.4);
        pseudoControlCostValue.push_back(0.3);
        pseudoControlCostValue.push_back(0.2);
        pseudoControlCostValue.push_back(0.1);
    };

    eveluatorResult calcQuality(inputTrajectory &trajectory, criterion method);

    void
    setControlCosts(std::function<double(double, double, double, double)> &func, std::vector<double> newMinJointsValues,
                    std::vector<double> newMaxJointValues) {
        controlCostFunction = func;
        maxJointsValues = newMaxJointValues;
        minJointsValues = newMinJointsValues;
    }

    void setControlCostsCallbackFunction(
            std::function<double(double, double, double, double)> &func) { controlCostFunction = func; }

    void setControlCosts(std::vector<double> newControlCosts) { pseudoControlCostValue = newControlCosts; }


    bool isTrajectoryBetterThan(inputTrajectory firstTraj, inputTrajectory secondTraj, criterion method);


private:
    std::vector<double> pseudoControlCostValue;
    std::vector<double> maxJointsValues;
    std::vector<double> minJointsValues;
    std::function<double(double, double, double, double)> controlCostFunction;

    double calculateJointDistance(inputTrajectory &trajectory);

    double calculateCartesianDistance(inputTrajectory &trajectory);

    double calculateJointJerk(inputTrajectory &trajectory);

    double calculateCartesianJerk(inputTrajectory &trajectory);

    double calculateJointMaxJerk(inputTrajectory &trajectory);

    double calculateCartesianMaxJerk(inputTrajectory &trajectory);

    double calculatePoseOrientationDistance(inputTrajectory &trajectory);

    double calculateControlCost(inputTrajectory &trajectory);

    trajectory_msgs::JointTrajectory makeJointDifference(trajectory_msgs::JointTrajectory &trajectory);

    std::vector <geometry_msgs::Pose> makeCartesianDifference(std::vector <geometry_msgs::Pose> &pose);

    double getQuaternionDotProduct(geometry_msgs::Quaternion orientation1, geometry_msgs::Quaternion orientation2);

    double getQuaternionDiffAngle(geometry_msgs::Quaternion orientation1, geometry_msgs::Quaternion orientation2);
};


#endif //PROJECT_TRAJECTORYEVALUATOR_H
