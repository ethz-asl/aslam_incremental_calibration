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

/** \file validateCamera.cpp
    \brief This file validates the camera intrinsics from a BAG file.
  */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include <sm/BoostPropertyTree.hpp>

#include "aslam/calibration/camera/CameraValidator.h"

using namespace aslam::calibration;
using namespace sm;

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file> <conf_file> "
      "<intrinsics_file>" << std::endl;
    return -1;
  }

  // loading configuration
  std::cout << "Loading configuration parameters..." << std::endl;
  BoostPropertyTree config;
  config.loadXml(argv[2]);

  if (config.getBool("camera/visualization"))
    cv::namedWindow("Image Results", CV_WINDOW_AUTOSIZE);

  // loading intrinsics
  std::cout << "Loading intrinsics..." << std::endl;
  BoostPropertyTree intrinsics;
  intrinsics.loadXml(argv[3]);

  // create the camera validator
  CameraValidator validator(PropertyTree(intrinsics, "intrinsics"),
    PropertyTree(config, "camera/validator"));

  // load ros bag file
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  const std::string rosTopic = config.getString("camera/rosTopic");
  topics.push_back(rosTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // processing ros bag file
  std::cout << "Processing BAG file..." << std::endl;
  size_t viewCounter = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->getTopic() == rosTopic) {
      sensor_msgs::ImagePtr image(it->instantiate<sensor_msgs::Image>());
      auto cvImage = cv_bridge::toCvCopy(image);
      validator.addImage(cvImage->image, image->header.stamp.toNSec());
      if (config.getBool("camera/visualization")) {
        cv::Mat resultImage;
        validator.getLastImage(resultImage);
        if (resultImage.data != NULL) {
          cv::imshow("Image Results", resultImage);
          cv::waitKey(1);
        }
        if (config.getBool("camera/validator/saveImages")) {
          if (!boost::filesystem::exists("images"))
            boost::filesystem::create_directory("images");
          std::stringstream stream;
          stream << "images/" << config.getString("camera/cameraId") << "-"
            << image->header.stamp.toNSec() << ".png";
          cv::imwrite(stream.str().c_str(), resultImage);
        }
      }
    }
  }

  // results
  std::cout << "reprojection error mean: "
    << validator.getReprojectionErrorMean().transpose() << std::endl;
  std::cout << "reprojection error standard deviation: "
    << validator.getReprojectionErrorStandardDeviation().transpose()
    << std::endl;
  std::cout << "max x reprojection error: "
    << validator.getReprojectionErrorMaxXError() << std::endl;
  std::cout << "max y reprojection error: "
    << validator.getReprojectionErrorMaxYError() << std::endl;
  std::cout << "number of outliers: " << validator.getNumOutliers()
    << std::endl;

  // output errors
  if (config.getBool("camera/validator/outputErrors")) {
    const std::vector<Eigen::Vector2d>& errors = validator.getErrors();
    std::ofstream errorsFile("errors.txt");
    errorsFile << std::fixed << std::setprecision(18);
    std::for_each(errors.cbegin(), errors.cend(), [&](
      decltype(*errors.cbegin()) x){errorsFile << x.transpose() << std::endl;});
    const std::vector<double>& errorsMd2 = validator.getMahalanobisDistances();
    std::ofstream errorsMd2File("errorsMd2.txt");
    errorsMd2File << std::fixed << std::setprecision(18);
    std::for_each( errorsMd2.cbegin(),  errorsMd2.cend(), [&](
      decltype(* errorsMd2.cbegin()) x){errorsMd2File << x << std::endl;});
  }

  return 0;
}
