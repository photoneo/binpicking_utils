/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */


#include <bin_pose_msgs/bin_pose_vector.h>
#include <binpicking_emulator/binpicking_emulator.h>
#include <pho_logger/operations_logger.h>

class Test {
  public:
    Test(ros::NodeHandle& nh) : logger_(), planner_(nh) {
    	    std::string path;
	    nh.getParam("/virtual_robot/log_path", path);
	    logger_.setPath(path);
	    logger_.setBufferSize(100);
    }  

    void setStartAndEnd(const BinpickingEmulator::JointValues &start, const BinpickingEmulator::JointValues &end) {
	    start_ = start; 
	    end_ = end;
 	    planner_.setStartState(start);
   }

    void run(int attempts) {
    std::ofstream outfile("/home/michaldobis/catkin_ws/datasets/time.txt");

	double total_time = 0;   
	for (int it =0; it < attempts; it++) {
		trajectory_msgs::JointTrajectory trajectory;
		auto start_time = ros::Time::now();
		auto planningResult = planner_.moveJ(end_, trajectory);
		double duration = (ros::Time::now() - start_time).toSec();		
		total_time += duration;
		outfile << std::to_string(duration) << "\n";
		double average_time = total_time/(it+1);
        	ROS_INFO("Iteration %d/%d take %f sec. Average time is %f", it, attempts, duration, average_time);
		if (planningResult == BinpickingEmulator::Result::OK) {
		 	logger_.createNewOperation();
			logger_.addTrajectory(trajectory.points, it, "CNT");
			logger_.saveLog();
		}
		if (!ros::ok()) break;
	    }
    }

    void run(ros::ServiceClient bin_pose_client) {
      bin_pose_client.waitForExistence();
      bin_pose_msgs::bin_pose_vector srv;
      bin_pose_client.call(srv);

      int it = 0;
      for (auto &target : srv.response.poses) {
	trajectory_msgs::JointTrajectory trajectory;
        auto planningResult = planner_.moveJ(target.grasp_pose, trajectory);
        ROS_INFO("Iteration %d/%d", it, srv.response.poses.size());
        if (planningResult == BinpickingEmulator::Result::OK) {
 	  logger_.createNewOperation();
	  logger_.addTrajectory(trajectory.points, it++, "CNT");
	  logger_.saveLog();
        }
        if (!ros::ok()) break;
    }
}
  private:
 OperationsLogger logger_;
BinpickingEmulator planner_;
BinpickingEmulator::JointValues start_;
BinpickingEmulator::JointValues end_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Test test(nh);
  BinpickingEmulator::JointValues start;
  BinpickingEmulator::JointValues end;

  double attempts;
  nh.getParam("/virtual_robot/start_pose", start);
  nh.getParam("/virtual_robot/end_pose", end);
  nh.getParam("/virtual_robot/attempts", attempts);
  test.setStartAndEnd(start, end);
  test.run(attempts);

    // Configure bin pose client
    //ros::ServiceClient bin_pose_client = node_handle.serviceClient<bin_pose_msgs::bin_pose_vector>("bin_pose");
    //test.run(bin_pose_client);
  ros::shutdown();
  return 0;
}
