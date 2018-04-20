/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Photoneo s.r.o.
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

#ifndef LOCALIZATION_INTERFACE_H
#define LOCALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <localization_interface/get_position.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <PhoLocalization.h>
#include <PhoXi.h>

class LocalizationNode
{
public:
  LocalizationNode(ros::NodeHandle* nh);
  ~LocalizationNode();

  bool localizationCallback(localization_interface::get_position::Request& req, localization_interface::get_position::Response& res);

private:

  // Consts
  double RAD2DEG = 57.2958;
  uint8_t LOCALIZATION_OK = 1;
  uint8_t LOCALIZATION_TIMEOUT = 255;
  double APPROACH_HEIGHT = 0.05;    //mm
  double DEAPPROACH_HEIGHT = 0.05;  //mm

  // Variables
  tf::TransformBroadcaster br_;
  pho::api::PhoXiFactory factory;
  pho::api::PPhoXi scanner;
  std::string plcf_filepath_;
  std::string scanner_id_;

  std::unique_ptr<pho::sdk::PhoLocalization> Localization;

};  // class

#endif // LOCALIZATION_INTERFACE_H
