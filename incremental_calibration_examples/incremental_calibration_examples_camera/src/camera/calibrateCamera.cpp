/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file calibrateCamera.cpp
    \brief This file calibrates the camera intrinsics from a BAG file.
  */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <sm/BoostPropertyTree.hpp>

#include "aslam/calibration/camera/CameraCalibrator.h"

using namespace aslam::calibration;
using namespace sm;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file> <conf_file>"
      << std::endl;
    return -1;
  }

  // loading configuration
  std::cout << "Loading configuration parameters..." << std::endl;
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[2]);

  // create the camera calibrator
  CameraCalibrator calibrator(PropertyTree(propertyTree, "camera/calibrator"));

  // load ros bag file
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  const std::string rosTopic = propertyTree.getString("camera/rosTopic");
  topics.push_back(rosTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // initializing geometry from the dataset
  std::cout << "Initializing geometry..." << std::endl;
  for (auto it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == rosTopic) {
      sensor_msgs::ImagePtr image(it->instantiate<sensor_msgs::Image>());
      auto cvImage = cv_bridge::toCvCopy(image);
      if (calibrator.initGeometry(cvImage->image))
        break;
    }
  }

  // processing ros bag file
  std::cout << "Processing BAG file..." << std::endl;
  size_t viewCounter = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->getTopic() == rosTopic) {
      sensor_msgs::ImagePtr image(it->instantiate<sensor_msgs::Image>());
      auto cvImage = cv_bridge::toCvCopy(image);
      calibrator.addImage(cvImage->image, image->header.stamp.toNSec());
    }
  }

  return 0;
}
