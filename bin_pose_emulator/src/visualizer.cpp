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


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "bin_pose_emulator/bin_pose_emulator.h"

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <visualization_msgs/MarkerArray.h>

class Visualizer : public BinPoseEmulator{
public:

    Visualizer(ros::NodeHandle* nh, std::string filepath) : BinPoseEmulator(nh, filepath) {

        /*uint32_t shape = visualization_msgs::Marker::ARROW;

        marker_.ns = "bin";
        marker_.id = 1;
        marker_.type = shape;
        marker_.action = visualization_msgs::Marker::ADD;

        marker_.scale.x = 0.01;
        marker_.scale.y = 0.02;
        marker_.scale.z = 0.05;

        marker_.color.r = 0.9f;
        marker_.color.g = 0.9f;
        marker_.color.b = 0.0f;
        marker_.color.a = 1.0;*/

    }

    void visualize(){

        geometry_msgs::Pose pose;
        geometry_msgs::Pose approach;

        std::ifstream infile;

        std::string str;

        infile.open("/home/controller/catkin_ws/fails.txt",  std::ifstream::in);

        while (!infile.eof()) {

            //position
            std::getline(infile, str);
            std::getline(infile, str);
            pose.position.x = getDouble(str);
            std::getline(infile, str);
            pose.position.y = getDouble(str);
            std::getline(infile, str);
            pose.position.z = getDouble(str);

            // orientation
            std::getline(infile, str);
            std::getline(infile, str);
            pose.orientation.x = getDouble(str);
            std::getline(infile, str);
            pose.orientation.y = getDouble(str);
            std::getline(infile, str);
            pose.orientation.z = getDouble(str);
            std::getline(infile, str);
            pose.orientation.w = getDouble(str);

            // empty line
            std::cout << pose << "\n";
            std::getline(infile, str);

            approach = pose;
            approach.position.z += 0.1;
            visualizeBin();
            visualizePose(pose, approach, true);

        }
    }

private:

    //visualization_msgs::Marker marker_;


    double getDouble(std::string str){
        std::vector<std::string> words;
        boost::split(words, str, boost::is_any_of(":"), boost::token_compress_on);
        return (double)atof(words[1].c_str());
    }

   /*void visualizePose(){

        marker_.header.frame_id = "/base_link";
        marker_.header.stamp = ros::Time::now();
      //  marker.lifetime = ros::Duration();
      //  marker_pub_.publish(marker);
    }

    void addPose(geometry_msgs::Pose grasp_pose,
                                  geometry_msgs::Pose approach_pose)
    {
        geometry_msgs::Point approach_point;
        approach_point.x = approach_pose.position.x;
        approach_point.y = approach_pose.position.y;
        approach_point.z = approach_pose.position.z;

        geometry_msgs::Point grasp_point;
        grasp_point.x = grasp_pose.position.x;
        grasp_point.y = grasp_pose.position.y;
        grasp_point.z = grasp_pose.position.z;

        marker_.points.push_back(approach_point);
        marker_.points.push_back(grasp_point);
    }*/

};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bin_pose_emulator");
  ros::NodeHandle nh;

  // Get config filepath from ROS Param server
  std::string filepath;
  nh.getParam("filepath", filepath);

    sleep(15);

    Visualizer visualizer(&nh, filepath);
    visualizer.visualize();


  ROS_WARN("creating service");
  ros::spin();

  return EXIT_SUCCESS;
}
