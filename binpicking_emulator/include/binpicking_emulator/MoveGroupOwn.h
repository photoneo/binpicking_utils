//
// Created by controller on 2/18/19.
//

#ifndef PROJECT_MOVEGROUPOWN_H
#define PROJECT_MOVEGROUPOWN_H

// MoveIt!
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit_msgs/MoveGroupGoal.h>


#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>

#include <moveit/move_group/move_group_context.h>

#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>


class MoveGroup{
public:
    MoveGroup(const boost::shared_ptr<tf::Transformer>& tf);
    MoveGroup(const boost::shared_ptr<tf::Transformer>& tf,planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monmult   );
    bool movej(const std::vector<double> start, const std::vector<double> goal, trajectory_msgs::JointTrajectory &trajectory, bool attached_object = false);
    bool movel(const std::vector<double> start, const std::vector<double> goal, trajectory_msgs::JointTrajectory &trajectory, moveit::core::GroupStateValidityCallbackFn *callback, double step = 0.01);
    bool movel(const geometry_msgs::Pose start, const geometry_msgs::Pose goal, trajectory_msgs::JointTrajectory &trajectory, moveit::core::GroupStateValidityCallbackFn *callback, double step = 0.01);
    void addCollisionObject(shapes::Mesh *objectMesh, std::string name);
    bool spawnObject(int id, const geometry_msgs::Pose &pose);
    bool removeObject(int id);
    bool attachObject(int id, const geometry_msgs::Pose &pose);
    bool detachObject();

    void visualizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory,float r=0.9,float g=0.9,float b=0.9);
    robot_model_loader::RobotModelLoaderPtr getRobotModelLoader();
    planning_scene::PlanningScenePtr getPlanningScene();

private:
    robot_model_loader::RobotModelLoader robot_model_loader_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_pipeline::PlanningPipeline planning_pipeline;
    moveit_msgs::MoveGroupGoal goal;
    move_group::MoveGroupContextPtr context_;



    ros::Publisher trajectory_pub_;
    ros::NodeHandle nh_;
    int attached_object_id_;

    std::vector<moveit_msgs::CollisionObject> collision_objects_;

    std::shared_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability> > capability_plugin_loader_;
    std::vector<move_group::MoveGroupCapabilityPtr> capabilities_;

    geometry_msgs::Pose getPose(const std::vector<double> &position);
    void configureCapabilities();
};

#endif //PROJECT_MOVEGROUPOWN_H
