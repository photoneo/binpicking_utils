//
// Created by controller on 2/15/19.
//

#ifndef PROJECT_KINEMATIC_H
#define PROJECT_KINEMATIC_H

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


class Kinematic{
public:
    Kinematic(robot_model::RobotModelPtr robot_model, planning_scene::PlanningScenePtr planning_scene = nullptr);
    bool getIK(const geometry_msgs::Pose &pose, std::vector<double> &joint_values);
    Eigen::Affine3d getFK(const std::vector<double> &joint_values);
    moveit::core::GroupStateValidityCallbackFn* getStateValidityCallback();

private:

    const int NUM_OF_ATTEMPTS = 5;
    const double TIMEOUT = 0.005;
    robot_state::RobotStatePtr kinematic_state;
  //  planning_scene::PlanningScenePtr planning_scene;
    moveit::core::GroupStateValidityCallbackFn groupStateValidityCallbackFn;

    robot_state::JointModelGroup* joint_model_group;

    bool isIKSolutionValid(const planning_scene::PlanningScenePtr planning_scene,
                      robot_state::RobotState* state,
                      const robot_model::JointModelGroup* jmg,
                      const double* ik_solution);
};
#endif //PROJECT_KINEMATIC_H
