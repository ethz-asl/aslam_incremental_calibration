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
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <sm/BoostPropertyTree.hpp>

#include <opencv2/highgui/highgui.hpp>

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
  BoostPropertyTree config;
  config.loadXml(argv[2]);

  if (config.getBool("camera/visualization")) {
    cv::namedWindow("Checkerboard Image", CV_WINDOW_AUTOSIZE);
  }

  // create the camera calibrator
  CameraCalibrator calibrator(PropertyTree(config, "camera/calibrator"));

  // load ros bag file
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  const std::string rosTopic = config.getString("camera/rosTopic");
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
      const size_t numObservations =
        calibrator.getEstimatorObservations().size();
      calibrator.addImage(cvImage->image, image->header.stamp.toNSec());
      if (config.getBool("camera/visualization")) {
        cv::Mat checkerboardImage;
        calibrator.getLastCheckerboardImage(checkerboardImage);
        cv::imshow("Checkerboard Image", checkerboardImage);
        cv::waitKey(1);
      }
      if (config.getBool("camera/calibrator/saveEstimatorImages")) {
        if (calibrator.getEstimatorObservations().size() != numObservations) {
          cv::Mat checkerboardImage;
          calibrator.getLastCheckerboardImage(checkerboardImage);
          std::stringstream stream;
          stream << "images/" << config.getString("camera/cameraId") << "-"
            << image->header.stamp.toNSec() << ".png";
          cv::imwrite(stream.str().c_str(), checkerboardImage);
        }
      }
    }
  }
  calibrator.processBatch();

  std::cout << "final parameters: " << std::endl;
  std::cout << "projection: " << calibrator.getProjection().transpose()
    << std::endl;
  std::cout << "projection standard deviation: "
    << calibrator.getProjectionStandardDeviation().transpose() << std::endl;
  std::cout << "distortion: " << calibrator.getDistortion().transpose()
    << std::endl;
  std::cout << "distortion standard deviation: "
    << calibrator.getDistortionStandardDeviation().transpose() << std::endl;
  std::cout << "null space: " << std::endl << calibrator.getNullSpace()
    << std::endl;
  std::cout << "initial cost: " << calibrator.getInitialCost() << std::endl;
  std::cout << "final cost: " << calibrator.getFinalCost() << std::endl;
  std::cout << "number of images for estimation: "
    << calibrator.getEstimatorObservations().size() << std::endl;
  std::cout << "total number of images: " << view.size() << std::endl;
  Eigen::VectorXd mean, variance, standardDeviation;
  double maxXError, maxYError;
  size_t numOutliers;
  calibrator.getStatistics(mean, variance, standardDeviation, maxXError,
    maxYError, numOutliers);
  std::cout << "reprojection error mean: " << mean.transpose()
    << std::endl;
  std::cout << "reprojection error standard deviation: " <<
    standardDeviation.transpose() << std::endl;
  std::cout << "max x reprojection error: " << maxXError << std::endl;
  std::cout << "max y reprojection error: " << maxYError << std::endl;
  std::cout << "number of outliers: " << numOutliers << std::endl;

  // write calibration to xml file
  BoostPropertyTree calibrationData("intrinsics");
  calibrator.write(calibrationData);
  calibrationData.saveXml(config.getString("camera/cameraId") + ".xml");

  return 0;
}
