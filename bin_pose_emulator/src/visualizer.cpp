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

#include <sensor_msgs/PointCloud.h>
#include <bin_pose_msgs/change_data.h>

#include <mutex>

class Visualizer{
public:

    Visualizer(ros::NodeHandle* nh, std::string filepath){

        point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud>("point_cloud",10);
        displayed_value_ = 4;

        std::ifstream infile;

        std::string str;

        infile.open("/home/controller/catkin_ws/planner_test/points_test_VW.txt",  std::ifstream::in);

        cloud_.points.clear();
        cloud_.channels.resize(3);

        ROS_WARN("visualizer start");
        while (!infile.eof()) {

            //position
            std::getline(infile, str);
            data_.push_back(parseLine(str));
        }

        data_.erase(data_.end());

        ROS_INFO("Size of values %d", data_[0].size());
        int max_value = 0;
        for (int i = 4; i < 14; i++){
            displayed_value_ = i;
            max_value = findMaxValue();
            ROS_INFO("Maximum fails [%s] on 1 point %d", data_types_[i].c_str(), i);
        }

        displayed_value_ = 4;
        max_value = findMaxValue();
        for (auto &values : data_){
            parseValues(cloud_, values, max_value);
        }
    }

    void visualize(){

        mutex_.lock();
        cloud_.header.stamp = ros::Time::now();
        cloud_.header.frame_id = "base_link";
        cloud_.header.seq++;
        point_cloud_pub_.publish(cloud_);
        mutex_.unlock();
   }

    bool changeDataCallback(bin_pose_msgs::change_data::Request &req, bin_pose_msgs::change_data::Response &res){

        displayed_value_ = req.data;

        int sum_of_failures = 0;
        mutex_.lock();
        res.max_failures_on_point = findMaxValue();

        for (auto &values : data_){
            sum_of_failures += parseValues(cloud_, values, res.max_failures_on_point);
        }

        res.message = "Type " + data_types_[displayed_value_] + " has " + std::to_string(sum_of_failures) + " fails";
        mutex_.unlock();

        return true;
    }
private:

    ros::Publisher point_cloud_pub_;
    sensor_msgs::PointCloud cloud_;
    int displayed_value_;
    std::vector<std::vector<double> > data_;
    std::mutex mutex_;

    std::map<int, std::string> data_types_= {
            {0, "ID"},
            {1,  "X"},
            {2,  "Y"},
            {3,  "Z"},
            {4,  "Grasp IK"},
            {5,  "Approach IK"},
            {6,  "Deapproach IK"},
            {7,  "start - approach J traj"},
            {8,  "aprroach - grasp L traj"},
            {9,  "grasp - deaprroach L traj"},
            {10, "deaprroach - end J traj"},
            {11,  "start - approach continuity"},
            {12,  "aprroach - grasp continuty"},
            {13,  "grasp - deaprroach continuity"},
            {14, "deaprroach - end continuity"},
    };

    std::vector<double> parseLine(std::string str){
        ROS_INFO("%s", str.c_str());
        std::vector<double> values;
        std::vector<std::string> words;
        boost::split(words, str, boost::is_any_of(","), boost::token_compress_on);

        for (const auto &word : words) {
           values.push_back((double) atof(word.c_str()));
        }
        return values;
    }

    int parseValues(sensor_msgs::PointCloud &cloud, const std::vector<double> &values, int max_intesity_value){

        geometry_msgs::Point32 p;
        sensor_msgs::ChannelFloat32 channel;

        p.x = values[1];
        p.y = values[2];
        p.z = values[3];

        double scale = 1.0/(max_intesity_value/2.0);
        float r = values[displayed_value_]*scale;
        float g = 0.0;
        float b = 2.0 - values[displayed_value_]*scale;

        cloud.channels[0].name = "r";
        cloud.channels[1].name = "g";
        cloud.channels[2].name = "b";
        cloud.channels[0].values.push_back(r);
        cloud.channels[1].values.push_back(b);
        cloud.channels[2].values.push_back(g);
        cloud.points.push_back(p);

        return values[displayed_value_];
    }

    int findMaxValue(){

        int max_value = 0;
        for (auto &values : data_){
            if (values[displayed_value_] > max_value)
                max_value = values[displayed_value_];
        }
        return max_value;
    }
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh;

  // Get config filepath from ROS Param server
  std::string filepath;
  nh.getParam("filepath", filepath);

    // Start Async Spinner with 2 threads
    ros::AsyncSpinner spinner(2);
    spinner.start();

    sleep(1);
    Visualizer visualizer(&nh, filepath);

    ros::ServiceServer change_data_service = nh.advertiseService("change_data", &Visualizer::changeDataCallback, &visualizer);


    ros::Rate rate(1);

    while (ros::ok()){
        visualizer.visualize();
        rate.sleep();
    }

  ROS_WARN("creating service");
  ros::spin();

  return EXIT_SUCCESS;
}
