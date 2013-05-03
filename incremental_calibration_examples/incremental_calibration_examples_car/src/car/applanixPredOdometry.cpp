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

/** \file applanixPredOdometry.cpp
    \brief This file predicts odometry measurements from Applanix data.
  */

#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include "aslam/calibration/car/ApplanixBinaryParser.h"

using namespace aslam::calibration;
using namespace sm::kinematics;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <LogFilename>" << std::endl;
    return -1;
  }

  // parse the Applanix log file
  std::cout << "Parsing Applanix data..." << std::endl;
  ApplanixBinaryParser applanixParser(argv[1]);
  applanixParser.parse();

  // car parameters
  const double L = 2.7; // wheelbase [m]
  const double e_r = 0.7575; // half-track rear [m]
  const double e_f = 0.7625; // half-track front [m]
  const double a0 = 0; // steering coefficient
  const double a1 = 1.0 / (M_PI / 180 / 10); // steering coefficient
  const double a2 = 0; // steering coefficient
  const double a3 = 0; // steering coefficient
  const double k_rl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_rr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fr = 1.0 / 3.6 / 100.0; // wheel coefficient

  // translation from odometry to IMU center
  Eigen::Vector3d t_io(0, 0, -0.785);

  // rotation from odometry to IMU center
  Eigen::Matrix3d C_io = Eigen::Matrix3d::Identity();

  // process Applanix data and generate odometry
  std::cout << "Creating predictive odometry..." << std::endl;
  std::vector<double> timestamps;
  std::vector<Eigen::Matrix<double, 5, 1> > odometry;
  timestamps.reserve(applanixParser.getNumNavigationSolution());
  odometry.reserve(applanixParser.getNumNavigationSolution());
  for (auto it = applanixParser.cbegin(); it != applanixParser.cend(); ++it) {
    timestamps.push_back(it->first);
    const Eigen::Vector3d v_iw(it->second.v_x, it->second.v_y, it->second.v_z);
    const Eigen::Vector3d om_ii(it->second.om_x, it->second.om_y,
      it->second.om_z);
    const EulerAnglesYawPitchRoll ypr;
    const Eigen::Matrix3d C_wi = ypr.parametersToRotationMatrix(
      Eigen::Vector3d(it->second.yaw, it->second.pitch, it->second.roll));
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
  std::ofstream applanixPredOdoMATLABFile("applanix-pred-odometry.txt");
  for (size_t i = 0; i < applanixParser.getNumNavigationSolution(); ++i)
    applanixPredOdoMATLABFile << std::fixed << std::setprecision(16)
      << timestamps[i] << " "
      << odometry[i].transpose() << std::endl;

  return 0;
}
