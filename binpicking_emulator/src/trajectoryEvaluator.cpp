//
// Created by martin on 21.10.2019.
//

#include "binpicking_emulator/trajectoryEvaluator.h"


eveluatorResult TrajectoryEvaluator::calcQuality(inputTrajectory &trajectory, criterion method) {

    eveluatorResult res;
    if (method == joinDistance || method == AllCriteria || method == JointCriteria) {
        res.jointDistance = calculateJointDistance(trajectory);
    }
    if (method == cartesianDistance || method == AllCriteria || method == CartesianCriteria) {
        res.cartesianDistance = calculateCartesianDistance(trajectory);
    }
    if (method == jointJerk || method == AllCriteria || method == JointCriteria) {
        res.jointJerk = calculateJointJerk(trajectory);
    }
    if (method == cartesianJerk || method == AllCriteria || method == CartesianCriteria) {
        res.cartesianJerk = calculateCartesianJerk(trajectory);
    }
    if (method == jointMaxJerk || method == AllCriteria || method == JointCriteria) {
        res.jointMaxJerk = calculateJointMaxJerk(trajectory);
    }
    if (method == cartesianMaxJerk || method == AllCriteria || method == CartesianCriteria) {
        res.cartesianMaxJerk = calculateCartesianMaxJerk(trajectory);
    }
    if (method == controlPseudoCost || method == AllCriteria || method == JointCriteria) {
        res.controlPseudoCost = calculateControlCost(trajectory);
    }
    if (method == PoseOrientationChange || method == AllCriteria || method == CartesianCriteria) {
        res.PoseOrientationDistance = calculatePoseOrientationDistance(trajectory);
    }
    return res;

}

double TrajectoryEvaluator::calculateJointDistance(inputTrajectory &trajectory) {

    trajectory_msgs::JointTrajectory dists = makeJointDifference(trajectory.jointPositions);
    double outputDist = 0;
    for (int i = 0; i < dists.points.size(); i++) {
        for (int h = 0; h < dists.points[i].positions.size(); h++) {
            outputDist += fabs(dists.points[i].positions[h]);
        }
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateCartesianDistance(inputTrajectory &trajectory) {
    std::vector <geometry_msgs::Pose> dists = makeCartesianDifference(trajectory.pose);
    double outputDist = 0;
    for (int i = 0; i < dists.size(); i++) {

        outputDist += sqrt(
                pow(dists[i].position.x, 2.0) + pow(dists[i].position.y, 2.0) + pow(dists[i].position.z, 2.0));
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateJointJerk(inputTrajectory &trajectory) {

    trajectory_msgs::JointTrajectory dists = makeJointDifference(trajectory.jointPositions);
    dists = makeJointDifference(dists);
    dists = makeJointDifference(dists);
    double outputDist = 0;
    for (int i = 0; i < dists.points.size(); i++) {
        for (int h = 0; h < dists.points[i].positions.size(); h++) {
            outputDist += fabs(dists.points[i].positions[h]);
        }
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateCartesianJerk(inputTrajectory &trajectory) {
    std::vector <geometry_msgs::Pose> dists = makeCartesianDifference(trajectory.pose);
    dists = makeCartesianDifference(dists);
    dists = makeCartesianDifference(dists);
    double outputDist = 0;
    for (int i = 0; i < dists.size(); i++) {

        outputDist += sqrt(
                pow(dists[i].position.x, 2.0) + pow(dists[i].position.y, 2.0) + pow(dists[i].position.z, 2.0));
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateJointMaxJerk(inputTrajectory &trajectory) {
    trajectory_msgs::JointTrajectory dists = makeJointDifference(trajectory.jointPositions);
    dists = makeJointDifference(dists);
    dists = makeJointDifference(dists);
    double outputDist = 0;
    for (int i = 0; i < dists.points.size(); i++) {
        for (int h = 0; h < dists.points[i].positions.size(); h++) {
            outputDist = outputDist < dists.points[i].positions[h] ? dists.points[i].positions[h] : outputDist;
        }
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateCartesianMaxJerk(inputTrajectory &trajectory) {
    std::vector <geometry_msgs::Pose> dists = makeCartesianDifference(trajectory.pose);
    dists = makeCartesianDifference(dists);
    dists = makeCartesianDifference(dists);
    double outputDist = 0;
    for (int i = 0; i < dists.size(); i++) {

        outputDist = outputDist <
                     sqrt(pow(dists[i].position.x, 2.0) + pow(dists[i].position.y, 2.0) + pow(dists[i].position.z, 2.0))
                     ? sqrt(
                        pow(dists[i].position.x, 2.0) + pow(dists[i].position.y, 2.0) + pow(dists[i].position.z, 2.0))
                     : outputDist;
    }

    return outputDist;
}

double TrajectoryEvaluator::calculateControlCost(inputTrajectory &trajectory) {

    trajectory_msgs::JointTrajectory dists = makeJointDifference(trajectory.jointPositions);
    double outputDist = 0;
    if (controlCostFunction && (dists.points[0].positions.size() == maxJointsValues.size() &&
                                dists.points[0].positions.size() == minJointsValues.size())) {

        for (int i = 0; i < dists.points.size(); i++) {
            for (int h = 0; h < dists.points[i].positions.size(); h++) {
                outputDist += controlCostFunction(trajectory.jointPositions.points[i + 1].positions[h],
                                                  dists.points[i].positions[h], minJointsValues[h], maxJointsValues[h]);
            }
        }
    } else {
        for (int i = 0; i < dists.points.size(); i++) {
            for (int h = 0; h < dists.points[i].positions.size(); h++) {
                outputDist += pseudoControlCostValue[h] * fabs(dists.points[i].positions[h]);
            }
        }
    }
    return outputDist;


}

double TrajectoryEvaluator::calculatePoseOrientationDistance(inputTrajectory &trajectory) {

    double res = 0;
    for (int i = 0; i < trajectory.pose.size() - 1; i++) {
        res += fabs(getQuaternionDiffAngle(trajectory.pose[i + 1].orientation, trajectory.pose[i].orientation));
    }

    return res;
}

trajectory_msgs::JointTrajectory
TrajectoryEvaluator::makeJointDifference(trajectory_msgs::JointTrajectory &trajectory) {
    trajectory_msgs::JointTrajectory res;
    for (int i = 0; i < trajectory.points.size() - 1; i++) {
        trajectory_msgs::JointTrajectoryPoint point;
        for (int h = 0; h < trajectory.points[i].positions.size(); h++) {
            point.positions.push_back(trajectory.points[i + 1].positions[h] - trajectory.points[i].positions[h]);
        }
        res.points.push_back(point);

    }

    return res;

}

std::vector <geometry_msgs::Pose>
TrajectoryEvaluator::makeCartesianDifference(std::vector <geometry_msgs::Pose> &pose) {
    std::vector <geometry_msgs::Pose> res;
    for (int i = 0; i < pose.size() - 1; i++) {
        geometry_msgs::Pose pss;
        pss.position.x = pose[i + 1].position.x - pose[i].position.x;
        pss.position.y = pose[i + 1].position.y - pose[i].position.y;
        pss.position.z = pose[i + 1].position.z - pose[i].position.z;


        pss.orientation.x = pose[i + 1].orientation.x - pose[i].orientation.x;
        pss.orientation.y = pose[i + 1].orientation.y - pose[i].orientation.y;
        pss.orientation.z = pose[i + 1].orientation.z - pose[i].orientation.z;
        pss.orientation.w = pose[i + 1].orientation.w - pose[i].orientation.w;
        res.push_back(pss);
    }
    return res;
}

double TrajectoryEvaluator::getQuaternionDotProduct(geometry_msgs::Quaternion orientation1,
                                                    geometry_msgs::Quaternion orientation2) {

    return orientation1.x * orientation2.x + orientation1.y * orientation2.y + orientation1.z * orientation2.z +
           orientation1.w * orientation2.w;
}

double TrajectoryEvaluator::getQuaternionDiffAngle(geometry_msgs::Quaternion orientation1,
                                                   geometry_msgs::Quaternion orientation2) {
    double t = getQuaternionDotProduct(orientation1, orientation2);
    if (t >= 1.0)
        return 0;
    return 2.0 * acos(t);
}

bool
TrajectoryEvaluator::isTrajectoryBetterThan(inputTrajectory firstTraj, inputTrajectory secondTraj, criterion method) {
    eveluatorResult result1 = calcQuality(firstTraj, method);
    eveluatorResult result2 = calcQuality(secondTraj, method);
    int compareCounter = 0;

    if (method == joinDistance || method == AllCriteria || method == JointCriteria) {
        if (result1.jointDistance < result2.jointDistance) {
            compareCounter += -1;
        } else if (result1.jointDistance == result2.jointDistance) {}
        else {
            compareCounter += 1;
        };
    }
    if (method == cartesianDistance || method == AllCriteria || method == CartesianCriteria) {
        if (result1.cartesianDistance < result2.cartesianDistance) {
            compareCounter += -1;
        } else if (result1.cartesianDistance == result2.cartesianDistance) {}
        else {
            compareCounter += 1;
        };

    }
    if (method == jointJerk || method == AllCriteria || method == JointCriteria) {
        if (result1.jointJerk < result2.jointJerk) {
            compareCounter += -1;
        } else if (result1.jointJerk == result2.jointJerk) {}
        else {
            compareCounter += 1;
        };

    }
    if (method == cartesianJerk || method == AllCriteria || method == CartesianCriteria) {
        if (result1.cartesianJerk < result2.cartesianJerk) {
            compareCounter += -1;
        } else if (result1.cartesianJerk == result2.cartesianJerk) {}
        else {
            compareCounter += 1;
        };

    }
    if (method == jointMaxJerk || method == AllCriteria || method == JointCriteria) {
        if (result1.jointMaxJerk < result2.jointMaxJerk) {
            compareCounter += -1;
        } else if (result1.jointMaxJerk == result2.jointMaxJerk) {}
        else {
            compareCounter += 1;
        };
    }
    if (method == cartesianMaxJerk || method == AllCriteria || method == CartesianCriteria) {
        if (result1.cartesianMaxJerk < result2.cartesianMaxJerk) {
            compareCounter += -1;
        } else if (result1.cartesianMaxJerk == result2.cartesianMaxJerk) {}
        else {
            compareCounter += 1;
        };
    }
    if (method == controlPseudoCost || method == AllCriteria || method == JointCriteria) {
        if (result1.controlPseudoCost < result2.controlPseudoCost) {
            compareCounter += -1;
        } else if (result1.controlPseudoCost == result2.controlPseudoCost) {}
        else {
            compareCounter += 1;
        };
    }
    if (method == PoseOrientationChange || method == AllCriteria || method == CartesianCriteria) {
        if (result1.PoseOrientationDistance < result2.PoseOrientationDistance) {
            compareCounter += -1;
        } else if (result1.PoseOrientationDistance == result2.PoseOrientationDistance) {}
        else {
            compareCounter += 1;
        };
    }
    return compareCounter < 0 ? true : false;
}