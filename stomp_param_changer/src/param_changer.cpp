/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Photoneo s.r.o.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Photoneo nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>

#include <stomp_param_changer/statistics.h>
#include <fstream>
#include <string>
/*struct noise_generator{

    std::string
};*/


struct stomp_parameters{
    int initialization_method;
    double control_cost_weight;
    int max_rollouts;
    int num_iterations;
    int num_iterations_after_valid;
    int num_rollouts;
    int num_timesteps;
};

class ParamChanger{
public:
    ParamChanger(std::string path){

        this->path = path;
        params.initialization_method = 1;
        params.control_cost_weight = 0.0;
        params.max_rollouts = 30;
        params.num_iterations = 200;
        params.num_iterations_after_valid = 0;
        params.num_rollouts = 20;
        params.num_timesteps = 5;


    }

    void restart(){

        ros::param::set("/move_group/stomp/manipulator/group_name", "manipulator");
        ros::param::set("/move_group/stomp/manipulator/optimization/control_cost_weight", params.control_cost_weight);
        ros::param::set("/move_group/stomp/manipulator/optimization/initialization_method", params.initialization_method);
        ros::param::set("/move_group/stomp/manipulator/optimization/max_rollouts", params.max_rollouts);
        ros::param::set("/move_group/stomp/manipulator/optimization/num_iterations", params.num_iterations);
        ros::param::set("/move_group/stomp/manipulator/optimization/num_iterations_after_valid", params.num_iterations_after_valid);
        ros::param::set("/move_group/stomp/manipulator/optimization/num_rollouts", params.num_rollouts);
        ros::param::set("/move_group/stomp/manipulator/optimization/num_timesteps", params.num_timesteps);

        /*ros::param::set("/move_group/stomp/manipulator/task/cost_functions", "manipulator");
        ros::param::set("/move_group/stomp/manipulator/task/noise_generator", "manipulator");
        ros::param::set("/move_group/stomp/manipulator/task/noisy_filters", "manipulator");
        ros::param::set("/move_group/stomp/manipulator/task/update_filters", "manipulator");*/

        system("roslaunch photoneo_virtual_robot_module abb_demo.launch &");

        ROS_ERROR("restarted");
    }

    void callback(const stomp_param_changer::statistics msg){

        static int num_of_callbacks = 0;

        std::to_string(num_of_callbacks);

        std::ofstream outfile_params(path + "/params_" + std::to_string(num_of_callbacks) + ".txt");
        std::ofstream outfile_results(path + "/results_" + std::to_string(num_of_callbacks++) + ".txt");

        ROS_ERROR("CALLBACK %s", path.c_str());
        //outfile_params << "PARAMS\n";
        outfile_params << "init_method " << params.initialization_method << "\n";
        outfile_params << "timesteps " << params.num_timesteps << "\n";
        outfile_params << "max_rollouts " << params.max_rollouts << "\n";
        outfile_params << "rollouts " << params.num_rollouts << "\n";
        outfile_params << "iterations " << params.num_iterations << "\n";
        outfile_params << "iterations_after_valid " << params.num_iterations_after_valid << "\n";
        outfile_params.close();

        //outfile_results << "RESULTS\n";
        outfile_results << msg << "\n";
//        outfile_results <<  "attempts " << msg.num_of_attempts << "\n";
//        outfile_results <<  "time " << msg.average_time << "\n";
//        outfile_results <<  "fails " << msg.num_of_fails << "\n";
//        outfile_results <<  "joint_differents " << msg.average_joint_diff << "\n";


        params.num_iterations += 10;
        //params.num_rollouts += 2;
       // params.max_rollouts += 2;

        outfile_results.close();

        restart();
    }

private:

    stomp_parameters params;
    std::string path;


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "param_changer");
    ros::NodeHandle nh("~");

    ParamChanger changer("/home/controller/catkin_ws/results");

    ros::Subscriber sub = nh.subscribe("/statistics", 1, &ParamChanger::callback, &changer);
    changer.restart();
    ros::spin();
    return 0;
}
