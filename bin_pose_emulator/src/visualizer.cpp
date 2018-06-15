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

class Visualizer : public BinPoseEmulator{
public:

    Visualizer(ros::NodeHandle* nh, std::string filepath) : BinPoseEmulator(nh, filepath) {

        this->filepath_config_ = filepath;
    }

    void visualize(){

        geometry_msgs::Pose pose;
        geometry_msgs::Pose approach;

        std::ifstream infile;

        std::string str;

        infile.open("/home/controller/catkin_ws/fails.txt",  std::ifstream::in);

        while (!infile.eof()){

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
            visualizePose(pose, approach);
            sleep(2);
        }

    }

private:
    std::string filepath_config_;
    std::string filepath_fails_;


    double getDouble(std::string str){
        std::vector<std::string> words;
        boost::split(words, str, boost::is_any_of(":"), boost::token_compress_on);
        return (double)atof(words[1].c_str());
    }

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
