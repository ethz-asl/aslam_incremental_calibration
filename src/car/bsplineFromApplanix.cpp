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

/** \file bsplineFromApplanix.cpp
    \brief This file computes a B-Spline from Applanix measurements
  */

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>

#include <bsplines/BSplinePose.hpp>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <LogFilename>" << std::endl;
    return -1;
  }

  // create specific rotation parameterization
  boost::shared_ptr<sm::kinematics::RotationVector>
    rv(new sm::kinematics::RotationVector);

  // create B-Spline
  const int order = 4;
  bsplines::BSplinePose bspline(order, rv);

  // parse log file
  std::cout << "Parsing log file..." << std::endl;
  std::vector<double> timestampsParse;
  std::vector<Eigen::Matrix<double, 6, 1> > posesParse;
  std::ifstream logFile(argv[1]);
  const sm::kinematics::EulerAnglesYawPitchRoll ypr;
  while (!logFile.eof()) {
    double timestamp;
    logFile >> timestamp;
    timestampsParse.push_back(timestamp);
    Eigen::Matrix<double, 6, 1> pose;
    logFile >> pose(0, 0) >> pose(1, 0) >> pose(2, 0)
      >> pose(3, 0) >> pose(4, 0) >> pose(5, 0);
    pose(3, 0) = sm::kinematics::angleMod(pose(3, 0));
    pose(4, 0) = sm::kinematics::angleMod(pose(4, 0));
    pose(5, 0) = sm::kinematics::angleMod(pose(5, 0));
    pose.tail<3>() = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(pose.tail<3>()));

    // ensure rotation vector do not flip
    if (!posesParse.empty()) {
      // previous rotation vector
      const Eigen::Matrix<double, 3, 1> pr =
        posesParse[posesParse.size() - 1].tail<3>();
      // current rotation vector
      const Eigen::Matrix<double, 3, 1> r = pose.tail<3>();
      // current angle
      const double angle = r.norm();
      // current axis
      const Eigen::Matrix<double, 3, 1> axis = r / angle;
      // best rotation vector
      Eigen::Matrix<double, 3, 1> best_r = r;
      // best distance
      double best_dist = (best_r - pr).norm();
      // find best vector
      for (int s = -3; s <= 4; ++s) {
        const Eigen::Matrix<double, 3, 1> aa = axis * (angle + M_PI * 2.0 * s);
        const double dist = (aa - pr).norm();
        if (dist < best_dist) {
          best_r = aa;
          best_dist = dist;
        }
      }
      pose.tail<3>() = best_r;
    }

    posesParse.push_back(pose);

    // skip trans. velocity
    logFile >> pose(0, 0) >> pose(1, 0) >> pose(2, 0);

    // skip rot. velocity
    logFile >> pose(0, 0) >> pose(1, 0) >> pose(2, 0);

    // skip acceleration
    logFile >> pose(0, 0) >> pose(1, 0) >> pose(2, 0);
  }
  timestampsParse.pop_back();
  posesParse.pop_back();

  // fill in data
  Eigen::VectorXd timestamps(timestampsParse.size());
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, timestampsParse.size());
  for (size_t i = 0; i < timestampsParse.size(); ++i) {
    timestamps(i) = timestampsParse[i];
    poses.col(i) = posesParse[i];
  }

  std::cout << "Number of measurements: " << timestampsParse.size()
    << std::endl;
  const double elapsedTime =
    timestampsParse[timestampsParse.size() - 1] - timestampsParse[0];
  std::cout << "Sequence length [s]: " << elapsedTime << std::endl;

  // fill the B-Spline with measurements
  const double lambda = 1e-6;
  const int measPerSec = timestampsParse.size() / elapsedTime;
  const int measPerSecDesired = 5;
  int numSegments;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = timestampsParse.size();
  std::cout << "Creating B-Spline with " << numSegments << " segments..."
    << std::endl;
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);

  // output poses from spline
  std::cout << "Outputting to file..." << std::endl;
  std::ofstream outFile("bsplinePoses.txt");
  for (size_t i = 0; i < timestampsParse.size(); ++i) {
    outFile << std::fixed << std::setprecision(16)
      << timestampsParse[i]
      << " " << bspline.position(timestampsParse[i]).transpose()
      << " " << ypr.rotationMatrixToParameters(
        bspline.orientation(timestampsParse[i])).transpose()
      << " " << bspline.linearVelocity(timestampsParse[i]).transpose()
      << " " << bspline.angularVelocityBodyFrame(timestampsParse[i]).transpose()
      << " " << bspline.linearAcceleration(timestampsParse[i]).transpose()
      << std::endl;
  }

  return 0;
}
