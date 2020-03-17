//
// Created by controller on 2/15/19.
//

#include "binpicking_emulator/kinematic.h"

Kinematic::Kinematic(robot_model::RobotModelPtr robot_model, planning_scene::PlanningScenePtr planning_scene) :
    kinematic_state(new robot_state::RobotState(robot_model))
{
    if (planning_scene == nullptr){
        planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    }
    //this->planning_scene = planning_scene;
    kinematic_state->setToDefaultValues();
    joint_model_group = robot_model->getJointModelGroup("manipulator");
    groupStateValidityCallbackFn = boost::bind(&Kinematic::isIKSolutionValid,this, planning_scene,_1,_2,_3);
}

bool Kinematic::getIK(const geometry_msgs::Pose &pose, std::vector<double> &joint_values) {

    joint_values.clear();
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, NUM_OF_ATTEMPTS, TIMEOUT, groupStateValidityCallbackFn);
    ROS_WARN("kinematic getIK vysledok: %d",found_ik);
    if (found_ik) kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    return found_ik;
}

bool Kinematic::isIKSolutionValid(const planning_scene::PlanningScenePtr planning_scene,
                                           robot_state::RobotState* state,
                                           const robot_model::JointModelGroup* jmg,
                                           const double* ik_solution)
{

    state->setJointGroupPositions(jmg, ik_solution);
    state->update();
    return (!planning_scene || !planning_scene->isStateColliding(*state, jmg->getName()));
}

moveit::core::GroupStateValidityCallbackFn *Kinematic::getStateValidityCallback(){
    return &groupStateValidityCallbackFn;
}
