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

/** \file applanixBSpline.cpp
    \brief This file computes a B-Spline from Applanix measurements
  */

#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>

#include <bsplines/BSplinePose.hpp>

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

  // output to MATLAB readable file
  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream applanixRawMATLABFile("applanix-raw.txt");
  applanixParser.writeMATLAB(applanixRawMATLABFile);

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
  const double lambda = 1e-6;
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

  // output data from spline
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
