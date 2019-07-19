/*********************************************************************
Copyright [2017] [Frantisek Durovsky]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 *********************************************************************/

#include "binpicking_emulator/binpicking_emulator.h"

using namespace pho_robot_loader;

BinpickingEmulator::BinpickingEmulator(ros::NodeHandle *nh) : trajectory_marker_index_(0),
                                                              outfile_fails_stomp_(
                                                                      "/home/controller/catkin_ws/planner_test/stomp_fails.txt"),
                                                              outfile_fails_ik_(
                                                                      "/home/controller/catkin_ws/planner_test/ik_fails.txt"),
                                                              outfile_joint_diff_(
                                                                      "/home/controller/catkin_ws/planner_test/joint_diff.txt"),
                                                              log_path_("/home/controller/catkin_ws/planner_test/"),
                                                              outfile_points_(
                                                                      "/home/controller/catkin_ws/planner_test/points.txt"),
                                                              path_length_test_(),
                                                              tf(new tf::TransformListener(ros::Duration(10.0)))/*,
                                                              planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf)),
                                                              groupTest(tf,planning_scene_monitor)*/
{

    ROS_ERROR("vosiel som do konstruktora");
    // Initialize Moveit group
    //group_.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

    // Load robot description
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

    // Load num of joints
    bool num_of_joints_success = nh->getParam("photoneo_module/num_of_joints", num_of_joints_);
    if (!num_of_joints_success) {
        ROS_WARN("Not able to load ""num_of_joints"" from param server, using default value 6");
        num_of_joints_ = 6;
    };
    ROS_ERROR("dosiel som do stredu konstruktora");
    planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
    groupTest = new MoveGroup(tf, planning_scene_monitor);
    ROS_ERROR("dosiel som do stredu konstruktora 2");

    for (int i = 0; i < THREADS_COUNT; i++) {
        groupOwn.push_back(new MoveGroup(tf, planning_scene_monitor));
    }
    // Resize start and end pose arrays
    start_pose_from_robot_.resize(num_of_joints_);
    end_pose_from_robot_.resize(num_of_joints_);

    // Configure bin pose client
    bin_pose_client_ = nh->serviceClient<bin_pose_msgs::bin_pose_vector>("bin_pose");

    // Set trajectory visualization publisher
    trajectory_pub_ = nh->advertise<visualization_msgs::Marker>("trajectory", 1);
    statistics_pub_ = nh->advertise<stomp_param_changer::statistics>("/statistics", 1);

    // Set move group params
    //group_->setPlannerId("RRTConnectkConfigDefault");
    //group_->setGoalTolerance(0.001);
    //group_->setNumPlanningAttempts(3);
    sum_time_ = 0;
    sum_traj_size_ = 0;
    num_of_attempt_ = 0;
    average_time_ = 0;
    average_traj_size_ = 0;
    num_of_fails_ = 0;
    num_of_success_ = 0;
    success_rate_ = 0;
    ik_fails_sum_ = 0;
    bad_trajectory_ = 0;
    ROS_ERROR("vysiel som z konstruktora");
    // outfile_time_ = new std::ofstream();
}

BinpickingEmulator::~BinpickingEmulator() {
}

bool BinpickingEmulator::binPickingInitCallback(photoneo_msgs::initialize_pose::Request &req,
                                                photoneo_msgs::initialize_pose::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Binpicking Init Service called");

    std::stringstream start_pose_string, end_pose_string;

    for (int i = 0; i < num_of_joints_; i++) {
        start_pose_from_robot_[i] = req.startPose.position[i];
        end_pose_from_robot_[i] = req.endPose.position[i];

        start_pose_string << req.startPose.position[i] << " ";
        end_pose_string << req.endPose.position[i] << " ";
    }

    ROS_INFO("BIN PICKING EMULATOR: START POSE: [%s] ", start_pose_string.str().c_str());
    ROS_INFO("BIN PICKING EMULATOR: END POSE: [%s]", end_pose_string.str().c_str());

    res.success = true;
    res.result = 0;
    return true;
}

bool BinpickingEmulator::binPickingScanCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Binpicking Scan Service called");
    ros::Duration(5).sleep();
    res.success = true;
    // res.message = "NOT_INITIALIZED";
    return true;
}

void BinpickingEmulator::binPickingLoopSimpleTraj() {

    bool use_linear_path = false;

    ros::NodeHandle nh;
    nh.getParam("/virtual_robot/start_pose", start_pose_from_robot_);
    nh.getParam("/virtual_robot/use_linear_path", use_linear_path);
    nh.getParam("/virtual_robot/log_path", log_path_);

    ROS_INFO("BIN PICKING EMULATOR: Binpicking Trajectory Service called");
    int grasp_traj_size, end_traj_size;
    moveit_msgs::RobotTrajectory to_grasp_pose;
    moveit_msgs::RobotTrajectory to_deapproach_pose;

    moveit::planning_interface::MoveGroupInterface::Plan to_end_pose;
    moveit::planning_interface::MoveGroupInterface::Plan to_approach_pose;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    planning_scene::PlanningScene planning_scene(kinematic_model);
    moveit::core::GroupStateValidityCallbackFn groupStateValidityCallbackFn = boost::bind(
            &BinpickingEmulator::isIKSolutionValid, this, &planning_scene, _1, _2, _3);

    // Get current state
    robot_state::RobotState current_state(*group_->getCurrentState());

    current_state.setJointGroupPositions(
            "manipulator", start_pose_from_robot_);

//    if (!path_length_test_.setStart(start_pose_from_robot_)){
//        ROS_ERROR("Can not compute start pose fk");
//         ros::shutdown();
//    }



    group_->setStartState(current_state);

    while (ros::ok()) {

        // Set Start state
        //---------------------------------------------------

        current_state.setJointGroupPositions(
                "manipulator", start_pose_from_robot_);
        group_->setStartState(current_state);

        // Get random bin picking pose from emulator
        //---------------------------------------------------

        bin_pose_msgs::bin_pose srv;
        geometry_msgs::Pose grasp_pose;

        if (bin_pose_client_.call(srv)) {
            grasp_pose = srv.response.grasp_pose;

        } else {
            // Dosiahol vsetky party v bin-e
            publishResult();
            ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/binpicking_emulator/save_log");
            std_srvs::Trigger srv;
            sleep(1);
            //   path_length_test_.saveLog();
            ros::shutdown();
        }

        std::vector<double> joint_values;
        bool found_ik = false;
        moveit::planning_interface::MoveItErrorCode success_approach = false;

        // Find IK in approach pose
        //---------------------------------------------------
        found_ik = kinematic_state->setFromIK(joint_model_group, grasp_pose, 5, 0.005, groupStateValidityCallbackFn);

        double time = 0;
        if (found_ik) {

            ROS_WARN("found IK");
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

            //---------------------------------------------------
            // Plan trajectory from current to approach pose
            //---------------------------------------------------

            double time = ros::Time::now().toSec();
            group_->setJointValueTarget(joint_values);
            success_approach = group_->plan(to_approach_pose);
            time = ros::Time::now().toSec() - time;

            createStatistics(success_approach, to_approach_pose.trajectory_.joint_trajectory, time, grasp_pose);
        } else {

            ROS_ERROR("IK failed");
            outfile_fails_ik_ << grasp_pose << "\n";
            ik_fails_sum_++;
        }

        if (success_approach) {
            // Get trajectory size from plan
            int traj_size = to_approach_pose.trajectory_.joint_trajectory.points.size();

            // SetStartState instead of trajectory execution
            current_state.setJointGroupPositions(
                    "manipulator", to_approach_pose.trajectory_.joint_trajectory.points[traj_size - 1].positions);
            group_->setStartState(current_state);

            // Visualize trajectory in RViz
            visualizeTrajectory(to_approach_pose.trajectory_.joint_trajectory);
        }
    }
}

void BinpickingEmulator::binPickingThreadsLoops() {

    std::vector<boost::thread *> threads;

    for(int i=0;i<THREADS_COUNT;i++)
    {
        threads.push_back(new boost::thread(boost::bind(&BinpickingEmulator::binPickingLoop,this,i)));
    }

    ros::waitForShutdown();
    for(int i=0;i<THREADS_COUNT;i++)
    {
        threads[i]->join();
    }
}


void BinpickingEmulator::binPickingLoop(int id) {
    bool use_linear_path = false;

    ros::NodeHandle nh;
    nh.getParam("/start_pose", start_pose_from_robot_);
    nh.getParam("/end_pose", end_pose_from_robot_);

    nh.getParam("/virtual_robot/use_linear_path", use_linear_path);
    nh.getParam("/virtual_robot/log_path", log_path_);

    ROS_INFO("BIN PICKING EMULATOR: Binpicking Trajectory Service called binPickingLoop");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    planning_scene::PlanningScene planning_scene(kinematic_model);
    moveit::core::GroupStateValidityCallbackFn groupStateValidityCallbackFn = boost::bind(
            &BinpickingEmulator::isIKSolutionValid, this, &planning_scene, _1, _2, _3);

    // Get current state
    ///robot_state::RobotState current_state(*group_->getCurrentState());

    ///current_state.setJointGroupPositions("manipulator", start_pose_from_robot_);

//    if (!path_length_test_.setStart(start_pose_from_robot_)){
//        ROS_ERROR("Can not compute start pose fk");
//        ros::shutdown();
//    }

    ///group_->setStartState(current_state);

    point_id_ = 0;
    last_point_.x = 0;
    last_point_.y = 0;
    last_point_.z = 0;

    ROS_ERROR("kinematic konst");
    Kinematic kinematic(groupOwn[id]->getRobotModelLoader()->getModel(), groupOwn[id]->getPlanningScene());

    ROS_ERROR("kinematic konst2");
    while (ros::ok()) {

        // Set Start state
        //---------------------------------------------------

        /*current_state.setJointGroupPositions(
                "manipulator", start_pose_from_robot_);
        group_->setStartState(current_state);
*/
        // Get random bin picking pose from emulator
        //---------------------------------------------------

        bin_pose_msgs::bin_pose_vector srv;
        ROS_ERROR("idem robit call");
        mutex_goals.lock();
        if (bin_pose_client_.call(srv)) {
            mutex_goals.unlock();
            std::vector <Waypoint> waypoints(5);
            ik_fails_.resize(waypoints.size());
            planner_fails_.resize(waypoints.size());
            continuity_checker_.resize(waypoints.size());
            ROS_ERROR("got response with %i poses",srv.response.poses.size());
            for (int zt = 0; zt < srv.response.poses.size(); zt++) {



                waypoints[0].joint_state = start_pose_from_robot_;
                waypoints[0].is_linear = false;
                waypoints[0].is_joint_space = true;
                waypoints[1].pose = srv.response.poses[zt].approach_pose;
                waypoints[1].is_linear = false;
                waypoints[1].is_joint_space = false;
                waypoints[2].pose = srv.response.poses[zt].grasp_pose;
                waypoints[2].is_linear = true;
                waypoints[2].is_joint_space = false;
                waypoints[3].pose = srv.response.poses[zt].deapproach_pose;
                waypoints[3].is_linear = true;
                waypoints[3].is_joint_space = false;
                waypoints[4].joint_state = end_pose_from_robot_;
                waypoints[4].is_linear = false;
                waypoints[4].is_joint_space = true;

                if (last_point_.x == 0 && last_point_.y == 0 && last_point_.z == 0) {
                    outfile_points_
                            << "X,  Y,  Z,  grasp IK, approach IK, deapproach IK, planner1, planner2, planner3, planner4, continuity1, continuity2\n";
                }
               /* if (last_point_.x != srv.response.grasp_pose.position.x ||
                    last_point_.y != srv.response.grasp_pose.position.y ||
                    last_point_.z != srv.response.grasp_pose.position.z) {
                    outfile_points_ << point_id_ << ", " << last_point_.x << ", " << last_point_.y << ", "
                                    << last_point_.z << ", "
                                    << ik_fails_[1] << ", " << ik_fails_[0] << ", " << ik_fails_[2] << ", "
                                    << planner_fails_[0] << ", " << planner_fails_[1] << ", " << planner_fails_[2]
                                    << ", " << planner_fails_[3] << ","
                                    << continuity_checker_[0] << ", " << continuity_checker_[1] << ", "
                                    << continuity_checker_[2] << ", " << continuity_checker_[3] << "\n";
                    point_id_++;
                    for (int i = 0; i < waypoints.size(); i++) {
                        ik_fails_[i] = 0;
                        planner_fails_[i] = 0;
                        continuity_checker_[i] = 0;
                    }

                    last_point_ = srv.response.grasp_pose.position;
                }*/


                std::vector<double> joint_values;
                bool found_ik = false;
                moveit::planning_interface::MoveItErrorCode success_approach = false;

                if (!kinematic.getIK(waypoints[2].pose, waypoints[2].joint_state)/*!kinematic_state->setFromIK(joint_model_group, waypoints[1].pose, "tool1", 3, 0.005,
                                              groupStateValidityCallbackFn)*/) {
                    ROS_ERROR("IK failed on point %d", point_id_);
                    outfile_fails_ik_ << "IK fail: " << 1 << "\n";
                    outfile_fails_ik_ << waypoints[1].pose << "\n";
                    ik_fails_sum_++;
                    ik_fails_[1]++;
                    continue;
                }

                for (int i = 1; i < waypoints.size(); i++) {


                    if (waypoints[i].is_joint_space) {
                        found_ik = true;
                    } else {
                        // Find IK in approach pose
                        //---------------------------------------------------
                        /*  found_ik = kinematic_state->setFromIK(joint_model_group, waypoints[i].pose, "tool1", 3, 0.005,
                                                                groupStateValidityCallbackFn);*/

                        found_ik = kinematic.getIK(waypoints[i].pose, waypoints[i].joint_state);
                    }
                    double time = 0;
                    if (found_ik) {

                        //ROS_WARN("found IK");


                        //---------------------------------------------------
                        // Plan trajectory from current to approach pose
                        //---------------------------------------------------

                        trajectory_msgs::JointTrajectory trajectory;
                        double time = ros::Time::now().toSec();

                        if (waypoints[i].is_linear) {
                            std::vector <geometry_msgs::Pose> linear_waypoints;
                            linear_waypoints.push_back(waypoints[i - 1].pose);
                            linear_waypoints.push_back(waypoints[i].pose);

                            success_approach = groupOwn[id]->movel(waypoints[i - 1].joint_state, waypoints[i].joint_state,
                                                                trajectory, kinematic.getStateValidityCallback());

                            // success_approach = group_->computeCartesianPath(linear_waypoints, 0.01, 0, trajectory, false);

                        } else {
                            //  moveit::planning_interface::MoveGroupInterface::Plan plan;
                            double time = ros::Time::now().toSec();

                            success_approach = groupOwn[id]->movej(waypoints[i - 1].joint_state, waypoints[i].joint_state,
                                                                trajectory);

                            //    success_approach = group_->plan(plan);
                            //    trajectory = plan.trajectory_;
                        }

                        time = ros::Time::now().toSec() - time;

                        if (!checkCartesianContinuity(trajectory, 2.2)) {

                            success_approach = false;
                            continuity_checker_[i]++;
                        }
                        createStatistics(success_approach, trajectory, time, waypoints[i].pose);

                        if (success_approach) {


                            // Get trajectory size from plan
                            int traj_size = trajectory.points.size();

                            // SetStartState instead of trajectory execution
                            /*  current_state.setJointGroupPositions(
                                      "manipulator", trajectory.joint_trajectory.points[traj_size - 1].positions);
                              group_->setStartState(current_state);*/


                            // Visualize trajectory in RViz
                            visualizeTrajectory(trajectory,id);

                        } else {
                            ROS_ERROR("Planner failed on waypoint %d", i);
                            planner_fails_[i]++;
                            break;
                        }

                    } else {

                        ROS_ERROR("IK failed on waypoint %d", i);
                        outfile_fails_ik_ << "IK fail: " << i << "\n";
                        outfile_fails_ik_ << waypoints[i].pose << "\n";
                        ik_fails_sum_++;
                        ik_fails_[i]++;
                        break;
                    }
                }
            }
            mutex_file.lock();
            //prebehlo cez vsetky orientacie..zapise vysledok
            outfile_points_ << point_id_ << ", " << last_point_.x << ", " << last_point_.y << ", "
                            << last_point_.z << ", "
                            << ik_fails_[1] << ", " << ik_fails_[0] << ", " << ik_fails_[2] << ", "
                            << planner_fails_[0] << ", " << planner_fails_[1] << ", " << planner_fails_[2]
                            << ", " << planner_fails_[3] << ","
                            << continuity_checker_[0] << ", " << continuity_checker_[1] << ", "
                            << continuity_checker_[2] << ", " << continuity_checker_[3] << "\n";
            mutex_file.unlock();
            point_id_++;
            for (int i = 0; i < waypoints.size(); i++) {
                ik_fails_[i] = 0;
                planner_fails_[i] = 0;
                continuity_checker_[i] = 0;
            }

            last_point_ = srv.response.poses[0].grasp_pose.position;
        } else {
            // Dosiahol vsetky party v bin-e
            publishResult();
            ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/binpicking_emulator/save_log");
            std_srvs::Trigger srv;
            sleep(1);
            //       path_length_test_.saveLog();
            ros::shutdown();
        }
    }
}

bool BinpickingEmulator::isIKSolutionValid(const planning_scene::PlanningScene *planning_scene,
                                           robot_state::RobotState *state,
                                           const robot_model::JointModelGroup *jmg,
                                           const double *ik_solution) {

    state->setJointGroupPositions(jmg, ik_solution);
    state->update();
    return (!planning_scene || !planning_scene->isStateColliding(*state, jmg->getName()));
}

void BinpickingEmulator::createStatistics(moveit::planning_interface::MoveItErrorCode success,
                                          trajectory_msgs::JointTrajectory trajectory, double time,
                                          const geometry_msgs::Pose pose) {

    num_of_attempt_++;
    double sum_joint_diff_swap = 0;
    std::vector<double> joints_diff;
    joints_diff.resize(6, 0);

    if (success) {
        for (int i = 1; i < trajectory.points.size(); i++) {
            for (int j = 0; j < 6; j++) {
                joints_diff[j] += fabs(trajectory.points[i].positions[j] -
                                       trajectory.points[i - 1].positions[j]);
                sum_joint_diff_swap += fabs(trajectory.points[i].positions[j] -
                                            trajectory.points[i - 1].positions[j]);
            }
        }

        for (int i = 0; i < joints_diff.size(); i++) {
            sum_joint_diff_ += joints_diff[i];
            //   path_length_test_.addJoint(i, joints_diff[i]);
        }

        // path_length_test_.addPoint(pose);
        //  path_length_test_.addPath(trajectory);

        //sum_joint_diff_ += sum_joint_diff_swap;
        sum_time_ += time;
        sum_traj_size_ += trajectory.points.size();
        num_of_success_++;
        average_time_ = sum_time_ / num_of_success_;
        average_joint_diff_ = sum_joint_diff_ / num_of_success_;
        average_traj_size_ = sum_traj_size_ / num_of_attempt_;
        success_rate_ = (double) (ik_fails_sum_ + num_of_fails_) / (num_of_attempt_ + ik_fails_sum_);

        outfile_joint_diff_ << num_of_success_ << " joint diff: " << sum_joint_diff_swap << " trajectory size: "
                            << trajectory.points.size() << "\n";

        if (sum_joint_diff_swap > 20.0) {
            outfile_joint_diff_ << pose << "\n";
            bad_trajectory_++;
        }

    } else {

        outfile_fails_stomp_ << pose << "\n";

        //outfile_fails_<< num_of_fails_ << ": " << pose << "\n";
        num_of_fails_++;

    }

    ROS_INFO("1. trajectory ik fails: %d planner fails: %d treshold %d", ik_fails_[0], planner_fails_[0],
             continuity_checker_[0]);
    ROS_INFO("2. trajectory ik fails: %d planner fails: %d treshold %d", ik_fails_[1], planner_fails_[1],
             continuity_checker_[1]);
    ROS_INFO("3. trajectory ik fails: %d planner fails: %d treshold %d", ik_fails_[2], planner_fails_[2],
             continuity_checker_[2]);
    ROS_INFO("4. trajectory ik fails: %d planner fails: %d treshold %d", ik_fails_[3], planner_fails_[3],
             continuity_checker_[3]);

    ROS_INFO(
            "attempt %d, average time %f, joint diff %f average joint diff %f, fails %d, average traj size %f, bad trajectory %d, traj size %d",
            num_of_attempt_, average_time_, sum_joint_diff_swap, average_joint_diff_, num_of_fails_, average_traj_size_,
            bad_trajectory_, trajectory.points.size());

    publishResult();
    writeToFile();
}

void BinpickingEmulator::publishResult() {

    stomp_param_changer::statistics msg;
    msg.num_of_attempts = num_of_attempt_;
    msg.num_of_fails = num_of_fails_;
    msg.average_joint_diff = average_joint_diff_;
    msg.average_time = average_time_;

    statistics_pub_.publish(msg);
}

void BinpickingEmulator::writeToFile() {

    stomp_param_changer::statistics msg;
    msg.num_of_attempts = num_of_attempt_;
    msg.num_of_fails = num_of_fails_;
    msg.average_joint_diff = average_joint_diff_;
    msg.average_time = average_time_;

    std::ofstream outfile_results(log_path_ + "/stomp_results.txt");

    outfile_results << msg << "\n";
    if (num_of_attempt_ % 100 == 0) {
        //     path_length_test_.saveLog();
    }
}

bool BinpickingEmulator::checkCartesianContinuity(trajectory_msgs::JointTrajectory &trajectory, float limit) {
    //static int bad_trajectory_counter = 0;

    for (int pointIdx = 1; pointIdx < trajectory.points.size(); pointIdx++) {

        for (int j = 0; j < 6; j++) {
            double diff = fabs(trajectory.points[pointIdx].positions[j] -
                               trajectory.points[pointIdx - 1].positions[j]);

            if (diff > limit) {
                ROS_ERROR("WRONG TRAJECTORY CONTINUITY diff: %lf on joit %d from %lf to %lf", diff, j,
                          trajectory.points[pointIdx - 1].positions[j],
                          trajectory.points[pointIdx].positions[j]);
                return false;
            }
        }
    }
    return true;
}

bool BinpickingEmulator::binPickingTrajCallback(photoneo_msgs::operations::Request &req,
                                                photoneo_msgs::operations::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Binpicking Trajectory Service called");

    int start_traj_size, approach_traj_size, grasp_traj_size, deapproach_traj_size, end_traj_size;
    moveit::planning_interface::MoveGroupInterface::Plan to_start_pose;
    moveit::planning_interface::MoveGroupInterface::Plan to_approach_pose;
    moveit_msgs::RobotTrajectory to_grasp_pose;
    moveit_msgs::RobotTrajectory to_deapproach_pose;
    moveit::planning_interface::MoveGroupInterface::Plan to_end_pose;

    // Get current state
    robot_state::RobotState current_state(*group_->getCurrentState());


    //---------------------------------------------------
    // Set Start state
    //---------------------------------------------------
    current_state.setJointGroupPositions(
            "manipulator", start_pose_from_robot_);

    group_->setStartState(current_state);

    // Get random bin picking pose from emulator
    bin_pose_msgs::bin_pose srv;
    geometry_msgs::Pose approach_pose, grasp_pose, deapproach_pose;

    if (bin_pose_client_.call(srv)) {
        grasp_pose = srv.response.grasp_pose;
        approach_pose = srv.response.approach_pose;
        deapproach_pose = srv.response.deapproach_pose;
    }

    //---------------------------------------------------
    // Plan trajectory to grasp pose
    //---------------------------------------------------
    std::vector <geometry_msgs::Pose> grasp_waypoints;
    grasp_waypoints.push_back(approach_pose);
    grasp_waypoints.push_back(grasp_pose);

    double success_grasp = group_->computeCartesianPath(grasp_waypoints, 0.02, 0, to_grasp_pose, false);
    ROS_INFO("Grasp Cartesian Path: %.2f%% achieved", success_grasp * 100.0);

    if (success_grasp == 1) {
        // Get trajectory size from plan
        grasp_traj_size = to_grasp_pose.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions(
                "manipulator", to_grasp_pose.joint_trajectory.points[grasp_traj_size - 1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_grasp_pose.joint_trajectory);
    }

    //---------------------------------------------------
    // Plan trajectory from grasp to deapproach pose
    //---------------------------------------------------
    std::vector <geometry_msgs::Pose> deapproach_waypoints;
    deapproach_waypoints.push_back(grasp_pose);
    deapproach_waypoints.push_back(deapproach_pose);

    double success_deapproach = group_->computeCartesianPath(deapproach_waypoints, 0.02, 0, to_deapproach_pose, false);
    ROS_INFO("Grasp Cartesian Path: %.2f%% achieved", success_deapproach * 100.0);

    if (success_deapproach == 1) {
        // Get trajectory size from plan
        deapproach_traj_size = to_deapproach_pose.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions(
                "manipulator", to_deapproach_pose.joint_trajectory.points[deapproach_traj_size - 1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_deapproach_pose.joint_trajectory);
    }


    //---------------------------------------------------
    // Plan trajectory to end pose
    //--------------------------------------------------
    double time = ros::Time::now().toSec();

    group_->setJointValueTarget(end_pose_from_robot_);
    moveit::planning_interface::MoveItErrorCode success_end = group_->plan(to_end_pose);
    time = ros::Time::now().toSec() - time;

    static std::ofstream outfile("/home/controller/catkin_ws/stomp_test.txt");


    if (success_end) {
        // Get trajectory size from plan
        end_traj_size = to_end_pose.trajectory_.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions("manipulator",
                                             to_end_pose.trajectory_.joint_trajectory.points[end_traj_size -
                                                                                             1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_end_pose.trajectory_.joint_trajectory);
        outfile << "true " << time << "\n";

        double sum_joint_diff_swap = 0;
        for (int i = 1; i < end_traj_size; i++) {
            for (int j = 0; j < 6; j++) {
                sum_joint_diff_swap += fabs(to_end_pose.trajectory_.joint_trajectory.points[i].positions[j] -
                                            to_end_pose.trajectory_.joint_trajectory.points[i - 1].positions[j]);
            }

        }
        sum_joint_diff_ = sum_joint_diff_swap / (6 * end_traj_size);


        sum_time_ += time;
        num_of_attempt_++;
        average_time_ = sum_time_ / num_of_attempt_;
        average_joint_diff_ = sum_joint_diff_ / num_of_attempt_;


    } else {
        outfile << "false " << time << "" << grasp_pose << "\n";
        num_of_fails_++;
    }

    //---------------------------------------------------
    // Compose binpicking as a sequence of operations
    //---------------------------------------------------

    // Check planning result
    //if ((success_approach) )//&& (success_grasp) && (success_deapproach) && (success_end))
    // {
    photoneo_msgs::operation binpicking_operation;

    // Operation 1 - Approach Trajectory
    binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_FINE;

    binpicking_operation.points.clear();
    for (int i = 0; i < approach_traj_size; i++)
        binpicking_operation.points.push_back(to_approach_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = 0;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 2 - Open Gripper
    binpicking_operation.operation_type = OPERATION::TYPE::GRIPPER;
    binpicking_operation.points.clear();
    binpicking_operation.gripper = GRIPPER::OPEN;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 3 - Grasp Trajectory
    binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_FINE;

    binpicking_operation.points.clear();
    for (int i = 0; i < grasp_traj_size; i++)
        binpicking_operation.points.push_back(to_grasp_pose.joint_trajectory.points[i]);

    binpicking_operation.gripper = 0;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 4 - Close Gripper
    binpicking_operation.operation_type = OPERATION::TYPE::GRIPPER;
    binpicking_operation.points.clear();
    binpicking_operation.gripper = GRIPPER::CLOSE;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 5 - Deapproach trajectory
    binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_FINE;

    binpicking_operation.points.clear();
    for (int i = 0; i < deapproach_traj_size; i++)
        binpicking_operation.points.push_back(to_deapproach_pose.joint_trajectory.points[i]);

    binpicking_operation.gripper = 0;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    // Operation 6 - End Trajectory
    binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_FINE;

    binpicking_operation.points.clear();
    for (int i = 0; i < end_traj_size; i++)
        binpicking_operation.points.push_back(to_end_pose.trajectory_.joint_trajectory.points[i]);

    binpicking_operation.gripper = 0;
    binpicking_operation.error = 0;
    binpicking_operation.info = 0;

    res.operations.push_back(binpicking_operation);

    return true;
    /* }
     else
     {
       return true;
     }*/
}

bool BinpickingEmulator::binPickingScanAndTrajCallback(photoneo_msgs::operations::Request &req,
                                                       photoneo_msgs::operations::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Binpicking Service called");

    int start_traj_size, approach_traj_size, grasp_traj_size, deapproach_traj_size, end_traj_size;
    moveit::planning_interface::MoveGroupInterface::Plan to_start_pose;
    moveit::planning_interface::MoveGroupInterface::Plan to_approach_pose;
    moveit_msgs::RobotTrajectory to_grasp_pose;
    moveit_msgs::RobotTrajectory to_deapproach_pose;
    moveit::planning_interface::MoveGroupInterface::Plan to_end_pose;

    // Get current state
    robot_state::RobotState current_state(*group_->getCurrentState());

    //---------------------------------------------------
    // Set Start state
    //---------------------------------------------------
    group_->setJointValueTarget(start_pose_from_robot_);
    group_->plan(to_start_pose);
    start_traj_size = to_start_pose.trajectory_.joint_trajectory.points.size();
    current_state.setJointGroupPositions(
            "manipulator", to_start_pose.trajectory_.joint_trajectory.points[start_traj_size - 1].positions);
    group_->setStartState(current_state);

    // Get random bin picking pose from emulator
    bin_pose_msgs::bin_pose srv;
    geometry_msgs::Pose approach_pose, grasp_pose, deapproach_pose;

    if (bin_pose_client_.call(srv)) {
        grasp_pose = srv.response.grasp_pose;
        approach_pose = srv.response.approach_pose;
        deapproach_pose = srv.response.deapproach_pose;
    }

    //---------------------------------------------------
    // Plan trajectory from current to approach pose
    //---------------------------------------------------
    group_->setPoseTarget(approach_pose);
    moveit::planning_interface::MoveItErrorCode success_approach = group_->plan(to_approach_pose);
    if (success_approach) {
        // Get trajectory size from plan
        approach_traj_size = to_approach_pose.trajectory_.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions(
                "manipulator", to_approach_pose.trajectory_.joint_trajectory.points[approach_traj_size - 1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_approach_pose.trajectory_.joint_trajectory);
    }

    //---------------------------------------------------
    // Plan trajectory from approach to grasp pose
    //---------------------------------------------------
    std::vector <geometry_msgs::Pose> grasp_waypoints;
    grasp_waypoints.push_back(approach_pose);
    grasp_waypoints.push_back(grasp_pose);

    double success_grasp = group_->computeCartesianPath(grasp_waypoints, 0.02, 0, to_grasp_pose, false);
    ROS_INFO("Grasp Cartesian Path: %.2f%% achieved", success_grasp * 100.0);

    if (success_grasp == 1) {
        // Get trajectory size from plan
        grasp_traj_size = to_grasp_pose.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions(
                "manipulator", to_grasp_pose.joint_trajectory.points[grasp_traj_size - 1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_grasp_pose.joint_trajectory);
    }

    //---------------------------------------------------
    // Plan trajectory from grasp to deapproach pose
    //---------------------------------------------------
    std::vector <geometry_msgs::Pose> deapproach_waypoints;
    deapproach_waypoints.push_back(grasp_pose);
    deapproach_waypoints.push_back(deapproach_pose);

    double success_deapproach = group_->computeCartesianPath(deapproach_waypoints, 0.02, 0, to_deapproach_pose, false);
    ROS_INFO("Grasp Cartesian Path: %.2f%% achieved", success_deapproach * 100.0);

    if (success_deapproach == 1) {
        // Get trajectory size from plan
        deapproach_traj_size = to_deapproach_pose.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions(
                "manipulator", to_deapproach_pose.joint_trajectory.points[deapproach_traj_size - 1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_deapproach_pose.joint_trajectory);
    }

    //---------------------------------------------------
    // Plan trajectory from deapproach to end pose
    //---------------------------------------------------
    group_->setJointValueTarget(end_pose_from_robot_);
    moveit::planning_interface::MoveItErrorCode success_end = group_->plan(to_end_pose);
    if (success_end) {
        // Get trajectory size from plan
        end_traj_size = to_end_pose.trajectory_.joint_trajectory.points.size();

        // SetStartState instead of trajectory execution
        current_state.setJointGroupPositions("manipulator",
                                             to_end_pose.trajectory_.joint_trajectory.points[end_traj_size -
                                                                                             1].positions);
        group_->setStartState(current_state);

        // Visualize trajectory in RViz
        visualizeTrajectory(to_end_pose.trajectory_.joint_trajectory);
    }

    //---------------------------------------------------
    // Compose binpicking as a sequence of operations
    //---------------------------------------------------

    // Check planning result
    if ((success_approach) && (success_grasp) && (success_deapproach) && (success_end)) {
        photoneo_msgs::operation binpicking_operation;

        // Operation 1 - Approach Trajectory
        binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_CNT;

        binpicking_operation.points.clear();
        for (int i = 0; i < approach_traj_size; i++)
            binpicking_operation.points.push_back(to_approach_pose.trajectory_.joint_trajectory.points[i]);

        binpicking_operation.gripper = GRIPPER::OPEN;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        // Operation 2 - Open Gripper
        binpicking_operation.operation_type = OPERATION::TYPE::GRIPPER;
        binpicking_operation.points.clear();
        binpicking_operation.gripper = GRIPPER::OPEN;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        // Operation 3 - Grasp Trajectory
        binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_FINE;

        binpicking_operation.points.clear();
        for (int i = 0; i < grasp_traj_size; i++)
            binpicking_operation.points.push_back(to_grasp_pose.joint_trajectory.points[i]);

        binpicking_operation.gripper = GRIPPER::OPEN;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        // Operation 4 - Close Gripper
        binpicking_operation.operation_type = OPERATION::TYPE::GRIPPER;
        binpicking_operation.points.clear();
        binpicking_operation.gripper = GRIPPER::CLOSE;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        // Operation 5 - Deapproach trajectory
        binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_CNT;

        binpicking_operation.points.clear();
        for (int i = 0; i < deapproach_traj_size; i++)
            binpicking_operation.points.push_back(to_deapproach_pose.joint_trajectory.points[i]);

        binpicking_operation.gripper = GRIPPER::CLOSE;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        // Operation 6 - End Trajectory
        binpicking_operation.operation_type = OPERATION::TYPE::TRAJECTORY_CNT;

        binpicking_operation.points.clear();
        for (int i = 0; i < end_traj_size; i++)
            binpicking_operation.points.push_back(to_end_pose.trajectory_.joint_trajectory.points[i]);

        binpicking_operation.gripper = GRIPPER::CLOSE;
        binpicking_operation.error = 0;
        binpicking_operation.info = 0;

        res.operations.push_back(binpicking_operation);

        return true;
    } else {
        return false;
    }
}

bool BinpickingEmulator::calibrationAddPointCallback(photoneo_msgs::add_point::Request &req,
                                                     photoneo_msgs::add_point::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Calibration Add Point Service called");

    ros::Duration(5).sleep();  // Simulating delay

    res.average_reprojection_error = 12.345;
    res.calibration_state = 0;
    res.too_close_indices = {0, 0, 0, 0};
    res.message = "OK";
    res.success = true;

    return true;
}

bool
BinpickingEmulator::calibrationSetToScannerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Calibration Set To Scanner Service called");

    ros::Duration(1).sleep();   // Simulating delay

    res.success = true;
    return true;
}

bool BinpickingEmulator::calibrationResetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("BIN PICKING EMULATOR: Calibration Reset Service called");

    ros::Duration(2).sleep();   // Simulating delay

    res.success = true;
    return true;
}

void BinpickingEmulator::visualizeTrajectory(trajectory_msgs::JointTrajectory trajectory, int color) {
    visualization_msgs::Marker marker;

    // Kinematic variables
    robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    marker.header.frame_id = "/base_link";
    marker.ns = "trajectory";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < trajectory.points.size(); i++) {
        kinematic_state->setJointGroupPositions("manipulator", trajectory.points[i].positions);
        const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool1");

        marker.header.stamp = ros::Time::now();
        marker.id = trajectory_marker_index_++;

        marker.pose.position.x = end_effector_state.translation()[0];
        marker.pose.position.y = end_effector_state.translation()[1];
        marker.pose.position.z = end_effector_state.translation()[2];

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = rs[color];
        marker.color.g = gs[color];
        marker.color.b = bs[color];
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(5);
        trajectory_pub_.publish(marker);
        ros::Duration(0.001).sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "binpicking_emulator");
    ros::NodeHandle nh;

    // Initial wait for Moveit to be properly loaded
    ros::Duration(3).sleep();

    // Wait for moveit services
    /* bool moveit_available = ros::service::exists("/compute_ik", true);
     while (!moveit_available)
     {
        ros::Duration(1).sleep();
        ROS_WARN("BIN PICKING EMULATOR: Waiting for Moveit Config to be properly loaded!");
        moveit_available = ros::service::exists("/compute_ik", true);
     }

   */  // Wait for bin_pose service
    bool bin_pose_emulator_available = ros::service::exists("/bin_pose", true);
    while (!bin_pose_emulator_available) {
        ros::Duration(1).sleep();
        bin_pose_emulator_available = ros::service::exists("/bin_pose", true);
        ROS_WARN("BIN PICKING EMULATOR: Waiting for Bin pose emulator to provide /bin_pose service ");
    }

    // Create BinpickingEmulator instance
    BinpickingEmulator emulator(&nh);

    // Advertise service
    /* ros::ServiceServer bin_picking_scan_service =
         nh.advertiseService(BINPICKING_SERVICES::SCAN, &BinpickingEmulator::binPickingScanCallback, &emulator);
     ros::ServiceServer bin_picking_traj_service =
         nh.advertiseService(BINPICKING_SERVICES::TRAJECTORY, &BinpickingEmulator::binPickingTrajCallback, &emulator);
   //  ros::ServiceServer bin_picking_scan_and_traj_service = nh.advertiseService(
    //     BINPICKING_SERVICES::SCAN_AND_TRAJECTORY, &BinpickingEmulator::binPickingScanAndTrajCallback, &emulator);
     ros::ServiceServer bin_picking_init_service =
         nh.advertiseService(BINPICKING_SERVICES::INITIALIZE, &BinpickingEmulator::binPickingInitCallback, &emulator);
     ros::ServiceServer calibration_add_point_service =
         nh.advertiseService(CALIBRATION_SERVICES::ADD_POINT, &BinpickingEmulator::calibrationAddPointCallback, &emulator);
     ros::ServiceServer calibration_set_to_scanner_service =
         nh.advertiseService(CALIBRATION_SERVICES::SET_TO_SCANNER, &BinpickingEmulator::calibrationSetToScannerCallback, &emulator);
     ros::ServiceServer calibration_reset_service =
         nh.advertiseService(CALIBRATION_SERVICES::RESET, &BinpickingEmulator::calibrationResetCallback, &emulator);
   */
    ROS_WARN("BIN PICKING EMULATOR: Ready");

    // Start Async Spinner with 2 threads
    /* ros::AsyncSpinner spinner(2);
     spinner.start();*/
   // emulator.binPickingLoop();
   emulator.binPickingThreadsLoops();
    ros::waitForShutdown();

    return EXIT_SUCCESS;
}
