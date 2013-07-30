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

/** \file optimizeOldSpline.cpp
    \brief This file optimizes the splines from a ROS bag file.
  */

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>

#include <sm/timing/TimestampCorrector.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/Optimizer2.hpp>

#include <bsplines/BSplinePose.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>

#include <libposlv/geo-tools/Geo.h>

#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/CarCalibrator.h"

using namespace aslam::calibration;
using namespace aslam::splines;
using namespace aslam::backend;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace bsplines;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file>" << std::endl;
    return -1;
  }
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  topics.push_back(std::string("/poslv/vehicle_navigation_solution"));
  topics.push_back(std::string("/poslv/vehicle_navigation_performance"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::cout << "Processing BAG file..." << std::endl;
  poslv::VehicleNavigationPerformanceMsgConstPtr lastVnp;
  bool firstVNS = true;
  double latRef = 0;
  double longRef = 0;
  double altRef = 0;
  size_t viewCounter = 0;
  CarCalibrator::ApplanixNavigationMeasurements measurements;
  TimestampCorrector<double> timestampCorrector;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->isType<poslv::VehicleNavigationPerformanceMsg>()) {
      poslv::VehicleNavigationPerformanceMsgConstPtr vnp(
        it->instantiate<poslv::VehicleNavigationPerformanceMsg>());
      lastVnp = vnp;
    }
    if (it->isType<poslv::VehicleNavigationSolutionMsg>()) {
      if (!lastVnp)
        continue;
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
      data.roll = deg2rad(vns->roll);
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
      data.x_sigma2 = lastVnp->eastPositionRMSError *
        lastVnp->eastPositionRMSError;
      data.y_sigma2 = lastVnp->northPositionRMSError *
        lastVnp->northPositionRMSError;
      data.z_sigma2 = lastVnp->downPositionRMSError *
        lastVnp->downPositionRMSError;
      data.roll_sigma2 = deg2rad(lastVnp->rollRMSError) *
        deg2rad(lastVnp->rollRMSError);
      data.pitch_sigma2 = deg2rad(lastVnp->pitchRMSError) *
        deg2rad(lastVnp->pitchRMSError);
      data.yaw_sigma2 = deg2rad(lastVnp->headingRMSError) *
        deg2rad(lastVnp->headingRMSError);
      data.v_x_sigma2 = lastVnp->eastVelocityRMSError *
        lastVnp->eastVelocityRMSError;
      data.v_y_sigma2 = lastVnp->northVelocityRMSError *
        lastVnp->northVelocityRMSError;
      data.v_z_sigma2 = lastVnp->downVelocityRMSError *
        lastVnp->downVelocityRMSError;
      measurements.push_back(
        std::make_pair(timestampCorrector.correctTimestamp(
        vns->timeDistance.time1, vns->header.stamp.toSec()), data));
    }
  }
  const size_t numMeasurements = measurements.size();
  Eigen::VectorXd timestamps(numMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
  const EulerAnglesYawPitchRoll ypr;
  auto rv = boost::make_shared<RotationVector>();
  for (size_t i = 0; i < measurements.size(); ++i) {
    Eigen::Vector3d crv = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(
      measurements[i].second.yaw,
      measurements[i].second.pitch,
      measurements[i].second.roll)));
    if (i > 0) {
      Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
      crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
    }
    timestamps(i) = measurements[i].first;
    Eigen::Matrix<double, 6, 1> pose;
    pose << measurements[i].second.x,
      measurements[i].second.y,
      measurements[i].second.z,
      crv;
    poses.col(i) = pose;
  }
  const double elapsedTime =
    timestamps[numMeasurements - 1] - timestamps[0];
  const int measPerSec = numMeasurements / elapsedTime;
  int numSegments;
  const double lambda = 1e-1;
  const int measPerSecDesired = 5;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = numMeasurements;
  BSplinePose bspline(4, rv);
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);

  std::cout << "Outputting spline data to MATLAB..." << std::endl;
  std::ofstream applanixSplineMATLABFile("applanix-spline.txt");
  for (size_t i = 0; i < numMeasurements; ++i)
    applanixSplineMATLABFile << std::fixed << std::setprecision(16)
      << timestamps(i) << " "
      << bspline.position(timestamps(i)).transpose() << " "
      << ypr.rotationMatrixToParameters(
        bspline.orientation(timestamps(i))).transpose() << " "
      << bspline.linearVelocity(timestamps(i)).transpose() << " "
      << bspline.angularVelocityBodyFrame(timestamps(i)).transpose() << " "
      << bspline.linearAccelerationBodyFrame(timestamps(i)).transpose()
      << std::endl;

  auto problem = boost::make_shared<OptimizationProblem>();
  auto bspdv = boost::make_shared<BSplinePoseDesignVariable>(bspline);
  for (size_t i = 0; i < bspdv->numDesignVariables(); ++i) {
    bspdv->designVariable(i)->setActive(true);
    problem->addDesignVariable(bspdv->designVariable(i), false);
  }
  for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
    ErrorTermPose::Input xm;
    xm.head<3>() = Eigen::Vector3d(it->second.x, it->second.y,
      it->second.z);
    xm.tail<3>() = Eigen::Vector3d(it->second.yaw, it->second.pitch,
      it->second.roll);
    ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
    Q(0, 0) = it->second.x_sigma2;
    Q(1, 1) = it->second.y_sigma2;
    Q(2, 2) = it->second.z_sigma2;
    Q(3, 3) = it->second.yaw_sigma2;
    Q(4, 4) = it->second.pitch_sigma2;
    Q(5, 5) = it->second.roll_sigma2;
    auto e_pose = boost::make_shared<ErrorTermPose>(
      bspdv->transformation(it->first), xm, Q);
    problem->addErrorTerm(e_pose);
  }
  Optimizer2Options options;
  options.verbose = true;
  options.doLevenbergMarquardt = false;
  options.linearSolver = "sparse_qr";
  SparseQRLinearSolverOptions linearSolverOptions;
  linearSolverOptions.colNorm = true;
  Optimizer2 optimizer(options);
  optimizer.getSolver<SparseQrLinearSystemSolver>()->setOptions(
    linearSolverOptions);
  optimizer.setProblem(problem);
  optimizer.optimize();

  // output optimized data from spline
  std::cout << "Outputting optimized spline data to MATLAB..." << std::endl;
  std::ofstream applanixSplineOptMATLABFile("applanix-spline-opt.txt");
  for (size_t i = 0; i < numMeasurements; ++i) {
    const Eigen::Vector3d r =
      bspdv->position(timestamps(i)).toEuclidean();
    const Eigen::Matrix3d C_wi =
      bspdv->orientation(timestamps(i)).toRotationMatrix();
    const Eigen::Vector3d v_iw =
      bspdv->linearVelocity(timestamps(i)).toEuclidean();
    const Eigen::Vector3d a_ii =
      bspdv->linearAccelerationBodyFrame(timestamps(i)).toEuclidean();
    const Eigen::Vector3d om_ii =
      bspdv->angularVelocityBodyFrame(timestamps(i)).toEuclidean();
    applanixSplineOptMATLABFile << std::fixed << std::setprecision(16)
      << timestamps(i) << " "
      << r.transpose() << " "
      << ypr.rotationMatrixToParameters(C_wi).transpose() << " "
      << v_iw.transpose() << " "
      << om_ii.transpose() << " "
      << a_ii.transpose() << std::endl;
  }

  return 0;
}
