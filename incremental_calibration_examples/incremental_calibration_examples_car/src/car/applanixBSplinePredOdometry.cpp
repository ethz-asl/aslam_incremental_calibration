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

/** \file applanixBSplinePredOdometry.cpp
    \brief This file predicts odometry measurements from Applanix B-spline data.
  */

#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <bsplines/BSplinePose.hpp>

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include "aslam/calibration/car/ApplanixBinaryParser.h"
#include "aslam/calibration/car/utils.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace bsplines;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <LogFilename>" << std::endl;
    return -1;
  }

  // parse the Applanix log file
  std::cout << "Parsing Applanix data..." << std::endl;
  ApplanixBinaryParser applanixParser(argv[1]);
  applanixParser.parse();

  // create rotation parameterizations
  boost::shared_ptr<RotationVector> rv(new RotationVector);
  const EulerAnglesYawPitchRoll ypr;

  // create B-Spline
  const int order = 4;
  BSplinePose bspline(order, rv);

  // iterate over the parsed messages
  std::cout << "Creating data for B-spline..." << std::endl;
  const size_t numMeasurements = applanixParser.getNumNavigationSolution();
  Eigen::VectorXd timestamps(numMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
  size_t i = 0;
  for (auto it = applanixParser.cbegin(); it != applanixParser.cend(); ++it) {
    Eigen::Vector3d crv = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(it->second.yaw,
      it->second.pitch, it->second.roll)));
    // ensure rotation vector do not flip
    if (i > 0) {
      Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
      crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
    }
    timestamps(i) = it->first;
    Eigen::Matrix<double, 6, 1> pose;
    pose << it->second.x, it->second.y, it->second.z, crv;
    poses.col(i) = pose;
    ++i;
  }

  std::cout << "Number of measurements: " << numMeasurements << std::endl;
  const double elapsedTime =
    timestamps(numMeasurements - 1) - timestamps(0);
  std::cout << "Sequence length [s]: " << elapsedTime << std::endl;

  // fill the B-Spline with measurements
  const double lambda = 1e-1;
  const int measPerSec = numMeasurements / elapsedTime;
  const int measPerSecDesired = 5;
  int numSegments;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = numMeasurements;
  std::cout << "Creating B-Spline with " << numSegments << " segments..."
    << std::endl;
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);

  // car parameters
  const double L = 2.698437452967974; // wheelbase [m]
  const double e_r = 0.744551686997985; // half-track rear [m]
  const double e_f = 0.7459111250452891; // half-track front [m]
  const double a0 = -0.8808883000114152; // steering coefficient
  const double a1 = 573.4542090947304587; // steering coefficient
  const double k_rl = 0.0028199619977176; // wheel coefficient
  const double k_rr = 0.0028260666435294; // wheel coefficient
  const double k_fl = 0.0028214039506398; // wheel coefficient
  const double k_fr = 0.0028223604206524; // wheel coefficient

  // translation from odometry to IMU center
  Eigen::Vector3d t_io(0, 0.0059385689655167, -0.785);

  // rotation from odometry to IMU center
  Eigen::Matrix3d C_io = Eigen::Matrix3d::Identity();

  // process Applanix B-spline data and generate odometry
  std::cout << "Creating predictive odometry..." << std::endl;
  std::vector<Eigen::Matrix<double, 5, 1> > odometry;
  odometry.reserve(applanixParser.getNumNavigationSolution());
  for (auto it = applanixParser.cbegin(); it != applanixParser.cend(); ++it) {
    const Eigen::Vector3d v_iw(bspline.linearVelocity(it->first));
    const Eigen::Vector3d om_ii(bspline.angularVelocityBodyFrame(it->first));
    const Eigen::Matrix3d C_wi(bspline.orientation(it->first));
    const Eigen::Vector3d v_ii = C_wi.transpose() * v_iw;
    const Eigen::Vector3d v_oo = C_io.transpose() * (v_ii + om_ii.cross(t_io));
    const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
    const double v_oo_x = v_oo(0);
    const double om_oo_z = om_oo(2);
    const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
    const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
    const double phi = atan(L * om_oo_z / v_oo_x);
    Eigen::Matrix<double, 5, 1> odo;
    odo(0) = v_oo_x - e_r * om_oo_z;
    odo(1) = v_oo_x + e_r * om_oo_z;
    odo(2) = (v_oo_x - e_f * om_oo_z) / cos(phi_L);
    odo(3) = (v_oo_x + e_f * om_oo_z) / cos(phi_R);
    odo(4) = phi;
    odometry.push_back(odo);
  }

  std::cout << "Outputting predictive odometry to MATLAB..." << std::endl;
  std::ofstream applanixPredOdoMATLABFile("applanix-spline-pred-odometry.txt");
  for (size_t i = 0; i < applanixParser.getNumNavigationSolution(); ++i)
    applanixPredOdoMATLABFile << std::fixed << std::setprecision(16)
      << timestamps(i) << " "
      << odometry[i].transpose() << std::endl;

  return 0;
}
