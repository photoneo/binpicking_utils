//
// Created by controller on 2/7/19.
//

#include "binpicking_emulator/MoveGroupOwn.h"

#include <moveit_msgs/MoveGroupResult.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/node_name.h>

#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>



MoveGroup::MoveGroup(const boost::shared_ptr<tf::Transformer>& tf) :
        nh_("~"),
        attached_object_id_(-1),
        robot_model_loader_("robot_description"),
        planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf)),
        planning_pipeline(planning_scene_monitor->getRobotModel())

{

    collision_objects_.clear();

    trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);
    if (!planning_scene_monitor->getPlanningScene()){
        ROS_ERROR("planning scene not configured");
        return;
    }

    goal.request.group_name = "manipulator";
    goal.request.num_planning_attempts = 1;
    goal.request.max_velocity_scaling_factor = 1.0;
    goal.request.max_acceleration_scaling_factor = 1.0;
    goal.request.allowed_planning_time = 13.0;
    //goal.request.planner_id = "RRTConnectkConfigDefault";
    //goal.request.planner_id = "TRRTkConfigDefault";

    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

   // configureCapabilities();
    sleep(0.02);
}

MoveGroup::MoveGroup(const boost::shared_ptr<tf::Transformer>& tf,planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_mon):
        nh_("~"),
        attached_object_id_(-1),
        robot_model_loader_("robot_description"),
        planning_pipeline(planning_scene_mon->getRobotModel())
{
    planning_scene_monitor=planning_scene_mon;
    collision_objects_.clear();

    trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);
    if (!planning_scene_monitor->getPlanningScene()){
        ROS_ERROR("planning scene not configured");
        return;
    }

    goal.request.group_name = "manipulator";
    goal.request.num_planning_attempts = 1;
    goal.request.max_velocity_scaling_factor = 1.0;
    goal.request.max_acceleration_scaling_factor = 1.0;
    goal.request.allowed_planning_time = 3.0;
    //goal.request.planner_id = "RRTConnectkConfigDefault";
   // goal.request.planner_id = "TRRTkConfigDefault";

    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // configureCapabilities();
    sleep(0.02);
}

bool MoveGroup::movej(const std::vector<double> start_pose, const std::vector<double> goal_pose, trajectory_msgs::JointTrajectory &trajectory , bool attached_object){

  //  planning_pipeline::PlanningPipeline planning_pipelineL(planning_scene_monitor->getRobotModel());
    trajectory.points.clear();

    planning_interface::MotionPlanResponse res;

    goal.request.workspace_parameters.header.frame_id = "/base_link";
    goal.request.workspace_parameters.header.stamp = ros::Time::now();
    goal.request.workspace_parameters.min_corner.x = -1;
    goal.request.workspace_parameters.min_corner.y = -1;
    goal.request.workspace_parameters.min_corner.z = -1;
    goal.request.workspace_parameters.max_corner.x = 1;
    goal.request.workspace_parameters.max_corner.y = 1;
    goal.request.workspace_parameters.max_corner.z = 1;

    robot_state::robotStateToRobotStateMsg(planning_scene_monitor->getPlanningScene()->getCurrentState(), goal.request.start_state);
    goal.request.start_state.joint_state.position = start_pose;

    moveit_msgs::Constraints constraints;
    constraints.joint_constraints.resize(6);

    for (int i = 0; i < 6; i++){

        constraints.joint_constraints[i].position = goal_pose[i];
        constraints.joint_constraints[i].joint_name = "joint_" + std::to_string(i + 1);
        constraints.joint_constraints[i].weight = 1.0;
        constraints.joint_constraints[i].tolerance_above = 0.0001;
        constraints.joint_constraints[i].tolerance_below = 0.0001;
    }

    goal.request.goal_constraints.clear();
    goal.request.goal_constraints.push_back(constraints);

   planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor);  // lock the scene so that it

   const planning_scene::PlanningSceneConstPtr& the_scene = static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene);

    planning_pipeline.generatePlan(the_scene/*planning_scene_monitor->getPlanningScene()*/, goal.request, res);

    goal.request.start_state.attached_collision_objects;

    auto trajectory_result = res.trajectory_;
    moveit_msgs::RobotTrajectory trajectory_msg;
    if (trajectory_result && !trajectory_result->empty())
    {
        trajectory_result->getRobotTrajectoryMsg(trajectory_msg);
        trajectory = trajectory_msg.joint_trajectory;
        return true;

    } else {

        ROS_ERROR("No trajectory found");
        return false;
    }
}

bool MoveGroup::movel(const std::vector<double> start, const std::vector<double> goal, trajectory_msgs::JointTrajectory &trajectory, moveit::core::GroupStateValidityCallbackFn *callback, double step){

   std::string link_name = "tool1";
    double jumb_treshold = 0;
    trajectory.points.clear();

    // Set start state
    robot_state::RobotState start_state = planning_scene_monitor->getPlanningScene()->getCurrentState();
    start_state.setJointGroupPositions("manipulator", start);

    // get joint model group
    if (const robot_model::JointModelGroup *jmg = start_state.getJointModelGroup("manipulator")) {
        // Set target
        Eigen::Affine3d target;
        tf::poseMsgToEigen(getPose(goal), target);

        // get carterian path
        std::vector<robot_state::RobotStatePtr> traj;
        double fraction = start_state.computeCartesianPath(jmg, traj, start_state.getLinkModel(link_name), target,
                                                           true,
                                                           step, jumb_treshold, *callback);

        // ROS_WARN("fraction %f", fraction);

        // Copy result to trajectory
        robot_trajectory::RobotTrajectory rt(planning_scene_monitor->getRobotModel(), "manipulator");
        for (std::size_t i = 0; i < traj.size(); ++i)
            rt.addSuffixWayPoint(traj[i], 0.0);
        moveit_msgs::RobotTrajectory robotTrajectory;
        rt.getRobotTrajectoryMsg(robotTrajectory);
        trajectory = robotTrajectory.joint_trajectory;

     /*   if (display_path && rt.getWayPointCount() > 0) {
            moveit_msgs::DisplayTrajectory disp;
            disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
            disp.trajectory.resize(1, robotTrajectory);
            robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
            display_path_.publish(disp);
        }*/

        return true;
    } else {
        return false;
    }
   // return movel(getPose(start), getPose(goal), trajectory, callback, step);
}

bool MoveGroup::movel(const geometry_msgs::Pose start, const geometry_msgs::Pose goal, trajectory_msgs::JointTrajectory &trajectory, moveit::core::GroupStateValidityCallbackFn *callback, double step){

//    trajectory.points.clear();
////    // Prepare waypoints
//    std::vector<geometry_msgs::Pose> linear_waypoints;
//    linear_waypoints.push_back(start);
//    linear_waypoints.push_back(goal);
//robot_state::GroupStateValidityCallbackFn constraint_fn = *callback;
////
////    // Make plan
////    moveit_msgs::RobotTrajectory robot_trajectory;
////    double ret = group_->computeCartesianPath(linear_waypoints, 0.01, 0, robot_trajectory, false);
////    if (ret == 1.0f) {
////        trajectory = robot_trajectory.joint_trajectory;
////        return true;
////    } else {
////        return false;
////    }
////    ROS_INFO("Received request to compute Cartesian path");
////    planning_scene_monitor->updateFrameTransforms();
////
//    robot_state::RobotState start_state =
//            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor->getCurrentState());
////   // robot_state::robotStateMsgToRobotState(req.start_state, start_state);
//    if (const robot_model::JointModelGroup* jmg = planning_scene_monitor->getPlanningScene()->getCurrentState().getJointModelGroup(req.group_name))
//    {
//        std::string link_name = "tool1";
////
//        bool ok = true;
//        EigenSTL::vector_Affine3d waypoints(2);
//        tf::poseMsgToEigen(linear_waypoints[0], waypoints[0]);
//        tf::poseMsgToEigen(linear_waypoints[1], waypoints[1]);
//
//        if (ok)
//        {
//            if (step < std::numeric_limits<double>::epsilon())
//            {
//                ROS_ERROR("Maximum step to take between consecutive configrations along Cartesian path was not specified (this "
//                          "value needs to be > 0)");
//                res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
//            }
//            else
//            {
//                if (waypoints.size() > 0)
//                {
//                    std::vector<robot_state::RobotStatePtr> traj;
//                    double fraction =
//                            start_state.computeCartesianPath(jmg, traj, start_state.getLinkModel(link_name), waypoints, true,
//                                                             step, 0, callback);
////                    robot_state::robotStateToRobotStateMsg(start_state, res.start_state);
////
//                    robot_trajectory::RobotTrajectory rt(planning_scene_monitor->getRobotModel(), "manipulator");
//                    for (std::size_t i = 0; i < traj.size(); ++i)
//                        rt.addSuffixWayPoint(traj[i], 0.0);
////
////                    // time trajectory
////                    // \todo optionally compute timing to move the eef with constant speed
////                    trajectory_processing::IterativeParabolicTimeParameterization time_param;
////                    time_param.computeTimeStamps(rt, 1.0);
////
////                    rt.getRobotTrajectoryMsg(res.solution);
////                    ROS_INFO("Computed Cartesian path with %u points (followed %lf%% of requested trajectory)",
////                             (unsigned int)traj.size(), res.fraction * 100.0);
////                    if (display_computed_paths_ && rt.getWayPointCount() > 0)
////                    {
////                        moveit_msgs::DisplayTrajectory disp;
////                        disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
////                        disp.trajectory.resize(1, res.solution);
////                        robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
////                        display_path_.publish(disp);
////                    }
//                }
//                return true;
//
//            }
//        }
//        else
//            return false;
//    }
//    else
//        return false;
//
//    return true;
}

void MoveGroup::addCollisionObject(shapes::Mesh *objectMesh, std::string name){

    // Create shape
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(objectMesh, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    // Create collision object
    moveit_msgs::CollisionObject object;
    object.meshes.resize(1);
    object.meshes[0] = co_mesh;               // Add shape
    object.id = name;                         // ID for spawning or attaching
    //object.header.frame_id = "base_link";
    //object.operation = object.ADD;
    collision_objects_.push_back(object);
}

bool MoveGroup::spawnObject(int id, const geometry_msgs::Pose &pose){

    // Set the position and orientation
    collision_objects_[id].mesh_poses.resize(1);
    collision_objects_[id].mesh_poses[0].position = pose.position;
    collision_objects_[id].mesh_poses[0].orientation = pose.orientation;
    collision_objects_[id].header.frame_id = "base_link";

    // Set operation as ADD
    collision_objects_[id].operation = collision_objects_[id].ADD;

    // Add to planning scene
    planning_scene_monitor->getPlanningScene()->processCollisionObjectMsg(collision_objects_[id]);
}

bool MoveGroup::removeObject(int id){

    // Set operation as REMOVE
    collision_objects_[id].operation = collision_objects_[id].REMOVE;

    // Remove from planning scene
    planning_scene_monitor->getPlanningScene()->processCollisionObjectMsg(collision_objects_[id]);
}

bool MoveGroup::attachObject(int id, const geometry_msgs::Pose &pose){

    if (attached_object_id_ >= 0){
        ROS_WARN("Object is already attached");
        return false;
    }
    attached_object_id_ = id;

    // Set the position and orientation
    collision_objects_[id].mesh_poses.resize(1);
    collision_objects_[id].mesh_poses[0].position = pose.position;
    collision_objects_[id].mesh_poses[0].orientation = pose.orientation;
    collision_objects_[id].header.frame_id = "tool1";

    // Set operation as ADD
    collision_objects_[id].operation = collision_objects_[id].ADD;

    // Create attached object
    moveit_msgs::AttachedCollisionObject obj;
    std::string link("gripper_base");
    obj.link_name = "tool1";
    obj.touch_links.push_back(link);
    obj.object = collision_objects_[id];

    // Add to planning scene
    planning_scene_monitor->getPlanningScene()->processAttachedCollisionObjectMsg(obj);
}

bool MoveGroup::detachObject(){

    // Set operation as REMOVE
    collision_objects_[attached_object_id_].operation = collision_objects_[attached_object_id_].REMOVE;
    
    // Create attached object
    moveit_msgs::AttachedCollisionObject obj;
    std::string link("gripper_base");
    obj.link_name = "tool1";
    obj.touch_links.push_back(link);
    obj.object = collision_objects_[attached_object_id_];
    
    // Add to planning scene   
    planning_scene_monitor->getPlanningScene()->processAttachedCollisionObjectMsg(obj);

    // Reset attached object id
    attached_object_id_ = -1;
}

void MoveGroup::visualizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory,float r,float g,float b)
{
    static int trajectory_marker_index = 0;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/base_link";

    marker.ns = "trajectory";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    ROS_WARN("Marker generator %i %0.2f %0.2f %0.2f",trajectory.points.size(),r,g,b);
    for (int i = 0; i < trajectory.points.size(); i++)
    {

        marker.header.stamp = ros::Time::now();
        marker.id = trajectory_marker_index++;

        marker.pose = getPose(trajectory.points[i].positions);
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(1);
        trajectory_pub_.publish(marker);
        ros::Duration(0.001).sleep();
    }
}

robot_model_loader::RobotModelLoaderPtr MoveGroup::getRobotModelLoader(){
    return std::make_shared<robot_model_loader::RobotModelLoader>(robot_model_loader_);
}

planning_scene::PlanningScenePtr MoveGroup::getPlanningScene(){
    return planning_scene_monitor->getPlanningScene();
}

geometry_msgs::Pose MoveGroup::getPose(const std::vector<double> &position){

    // Kinematic variables
    robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    kinematic_state->setJointGroupPositions("manipulator", position);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool1");

    geometry_msgs::Pose pose;
    pose.position.x = end_effector_state.translation()[0];
    pose.position.y = end_effector_state.translation()[1];
    pose.position.z = end_effector_state.translation()[2];

    auto rotation_matrix = end_effector_state.rotation();
    // convert rotation matrix to tf matrix
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(rotation_matrix(0,0)), static_cast<double>(rotation_matrix(0,1)), static_cast<double>(rotation_matrix(0,2)),
                  static_cast<double>(rotation_matrix(1,0)), static_cast<double>(rotation_matrix(1,1)), static_cast<double>(rotation_matrix(1,2)),
                  static_cast<double>(rotation_matrix(2,0)), static_cast<double>(rotation_matrix(2,1)), static_cast<double>(rotation_matrix(2,2)));

    // Convert to quternion
    tf::Quaternion quaternion;
    tf3d.getRotation(quaternion);

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z= quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}


void MoveGroup::configureCapabilities()
{
    using namespace move_group;

    context_.reset(new MoveGroupContext(planning_scene_monitor, true, true));

    try
    {
        capability_plugin_loader_.reset(
                new pluginlib::ClassLoader<MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities: " << ex.what());
        return;
    }

    std::set<std::string> capabilities;

    // add default capabilities
    for (size_t i = 0; i < sizeof(DEFAULT_CAPABILITIES) / sizeof(DEFAULT_CAPABILITIES[0]); ++i)
        capabilities.insert(DEFAULT_CAPABILITIES[i]);

    // add capabilities listed in ROS parameter
    std::string capability_plugins;
    if (nh_.getParam("capabilities", capability_plugins))
    {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
        capabilities.insert(tok.begin(), tok.end());
    }

    // drop capabilities that have been explicitly disabled
    if (nh_.getParam("disable_capabilities", capability_plugins))
    {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
        for (boost::tokenizer<boost::char_separator<char> >::iterator cap_name = tok.begin(); cap_name != tok.end();
             ++cap_name)
            capabilities.erase(*cap_name);
    }

    for (std::set<std::string>::iterator plugin = capabilities.cbegin(); plugin != capabilities.cend(); ++plugin)
    {
        try
        {
            printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, plugin->c_str());
            MoveGroupCapability* cap = capability_plugin_loader_->createUnmanagedInstance(*plugin);
            cap->setContext(context_);
            cap->initialize();
            capabilities_.push_back(MoveGroupCapabilityPtr(cap));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_ERROR_STREAM("Exception while loading move_group capability '"
                                     << *plugin << "': " << ex.what() << std::endl
                                     << "Available capabilities: "
                                     << boost::algorithm::join(capability_plugin_loader_->getDeclaredClasses(), ", "));
        }
    }

    std::stringstream ss;
    ss << std::endl;
    ss << std::endl;
    ss << "********************************************************" << std::endl;
    ss << "* MoveGroup using: " << std::endl;
    for (std::size_t i = 0; i < capabilities_.size(); ++i)
        ss << "*     - " << capabilities_[i]->getName() << std::endl;
    ss << "********************************************************" << std::endl;
    ROS_INFO_STREAM(ss.str());
}
