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

/** \file processBagOldSpline.cpp
    \brief This file estimates the different covariances from a ROS bag file.
  */

#include <iostream>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/rotations.hpp>

#include <bsplines/BSplinePose.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>

#include <libposlv/sensor/Utils.h>
#include <libposlv/geo-tools/Geo.h>

#include "aslam/calibration/car/CarCalibrator.h"
#include "aslam/calibration/car/utils.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace bsplines;


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0]
      << " <ros_bag_file>" << std::endl;
    return -1;
  }
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  topics.push_back(std::string("/can_prius/front_wheels_speed"));
  topics.push_back(std::string("/can_prius/rear_wheels_speed"));
  topics.push_back(std::string("/can_prius/steering1"));
  topics.push_back(std::string("/poslv/vehicle_navigation_solution"));
  topics.push_back(std::string("/poslv/vehicle_navigation_performance"));
  topics.push_back(std::string("/poslv/time_tagged_dmi_data"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  CarCalibrator::ApplanixNavigationMeasurements applanixNavigationMeasurements;
  bool firstVNS = true;
  double latRef = 0;
  double longRef = 0;
  double altRef = 0;
  std::cout << "Reading BAG file for Applanix navigation measurements..."
    << std::endl;
  size_t viewCounter = 0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->isType<poslv::VehicleNavigationSolutionMsg>()) {
      poslv::VehicleNavigationSolutionMsgConstPtr vns(
        it->instantiate<poslv::VehicleNavigationSolutionMsg>());
      if (firstVNS) {
        latRef = vns->latitude;
        longRef = vns->longitude;
        altRef = vns->altitude;
        firstVNS = false;
      }
      double x_ecef, y_ecef, z_ecef;
      Geo::wgs84ToEcef(vns->latitude, vns->longitude, vns->altitude, x_ecef,
        y_ecef, z_ecef);
      double x_enu, y_enu, z_enu;
      Geo::ecefToEnu(x_ecef, y_ecef, z_ecef, latRef, longRef, altRef, x_enu,
        y_enu, z_enu);
      CarCalibrator::ApplanixNavigationMeasurement data;
      data.x = x_enu;
      data.y = y_enu;
      data.z = z_enu;
      data.yaw = angleMod(deg2rad(-vns->heading) + M_PI / 2);
      data.pitch = deg2rad(-vns->pitch);
      data.roll = deg2rad(-vns->roll);
      Eigen::Vector3d linearVelocity =
        Geo::R_ENU_NED::getInstance().getMatrix() * Eigen::Vector3d(
        vns->northVelocity, vns->eastVelocity, vns->downVelocity);
      data.v_x = linearVelocity(0);
      data.v_y = linearVelocity(1);
      data.v_z = linearVelocity(2);
      data.om_x = deg2rad(vns->angularRateLong);
      data.om_y = -deg2rad(vns->angularRateTrans);
      data.om_z = -deg2rad(vns->angularRateDown);
      data.a_x = vns->accLong;
      data.a_y = -vns->accTrans;
      data.a_z = -vns->accDown;
      data.v = vns->speed;
      applanixNavigationMeasurements.push_back(
        std::make_pair(vns->header.stamp.toSec(), data));
    }
  }
  std::cout << std::endl;
  std::cout << "Building spline..." << std::endl;
  const size_t numMeasurements = applanixNavigationMeasurements.size();
  Eigen::VectorXd timestamps(numMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
  const EulerAnglesYawPitchRoll ypr;
  auto rv = boost::make_shared<RotationVector>();
  std::ofstream crvFile("crv.txt");
  for (size_t i = 0; i < numMeasurements; ++i) {
    Eigen::Vector3d crv = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(
      applanixNavigationMeasurements[i].second.yaw,
      applanixNavigationMeasurements[i].second.pitch,
      applanixNavigationMeasurements[i].second.roll)));
    if (i > 0) {
      Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
      crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
      crvFile << crv.transpose() << std::endl;
    }
    timestamps(i) = applanixNavigationMeasurements[i].first;
    Eigen::Matrix<double, 6, 1> pose;
    pose << applanixNavigationMeasurements[i].second.x,
      applanixNavigationMeasurements[i].second.y,
      applanixNavigationMeasurements[i].second.z,
      crv;
    poses.col(i) = pose;
  }
  const double elapsedTime =
    timestamps[numMeasurements - 1] - timestamps[0];
  const int measPerSec = numMeasurements / elapsedTime;
  int numSegments;
  const double lambda = 0;
  const int measPerSecDesired = 5;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = numMeasurements;
  BSplinePose bspline(4, rv);
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);
  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream applanixRawMATLABFile("applanix-raw.txt");
  for (auto it = applanixNavigationMeasurements.cbegin();
      it != applanixNavigationMeasurements.cend(); ++it)
    applanixRawMATLABFile << std::fixed << std::setprecision(16)
      << it->first << " "
      << it->second.x << " " << it->second.y << " " << it->second.z << " "
      << it->second.yaw << " " << it->second.pitch << " "
      << it->second.roll << " "
      << it->second.v_x << " " << it->second.v_y << " " << it->second.v_z << " "
      << it->second.om_x << " " << it->second.om_y << " "
      << it->second.om_z << " "
      << it->second.a_x << " " << it->second.a_y << " " << it->second.a_z
      << std::endl;
  std::cout << "Outputting spline data to MATLAB..." << std::endl;
  std::ofstream applanixSplineMATLABFile("applanix-spline.txt");
  for (size_t i = 0; i < numMeasurements; ++i) {
    applanixSplineMATLABFile << std::fixed << std::setprecision(16)
      << timestamps(i) << " "
      << bspline.position(timestamps(i)).transpose() << " "
      << ypr.rotationMatrixToParameters(
        bspline.orientation(timestamps(i))).transpose() << " "
      << bspline.linearVelocity(timestamps(i)).transpose() << " "
      << bspline.angularVelocityBodyFrame(timestamps(i)).transpose() << " "
      << bspline.linearAccelerationBodyFrame(timestamps(i)).transpose()
      << std::endl;
  }
  return 0;
}
