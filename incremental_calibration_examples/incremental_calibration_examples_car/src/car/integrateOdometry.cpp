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

/** \file integrateOdometry.cpp
    \brief This file integrates the odometry.
  */

#include <iostream>
#include <fstream>

#include <bsplines/BSpline.hpp>

#include "aslam/calibration/car/CANBinaryParser.h"
#include "aslam/calibration/car/Odometry.h"
#include "aslam/calibration/car/AckermanOdometry.h"
#include "aslam/calibration/car/ApplanixBinaryParser.h"

using namespace aslam::calibration;
using namespace bsplines;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
      << " <applanix-binary-log> <can-binary-log>" << std::endl;
    return -1;
  }

  // parse the Applanix log file
  std::cout << "Parsing Applanix data to get starting pose..." << std::endl;
  ApplanixBinaryParser applanixParser(argv[1]);
  applanixParser.parse();
  double yaw0 = applanixParser.cbegin()->second.yaw;

  // parse the CAN log file
  std::cout << "Parsing CAN data..." << std::endl;
  CANBinaryParser canParser(argv[2]);
  canParser.parse();

  // create B-Spline for front wheel speeds
  std::cout << "Creating B-spline for front wheel speeds..." << std::endl;
  const int orderFws = 4;
  BSpline bsplineFws(orderFws);
  const size_t numFwsMeasurements = canParser.getNumFrontWheels();
  Eigen::VectorXd timestampsFws(numFwsMeasurements);
  Eigen::MatrixXd pointsFws(2, numFwsMeasurements);
  size_t i = 0;
  for (auto it = canParser.cbeginFw(); it != canParser.cendFw(); ++it) {
    timestampsFws(i) = it->first;
    pointsFws.col(i) = Eigen::Vector2d(it->second.first, it->second.second);
    ++i;
  }
  const double elapsedTimeFws = timestampsFws(numFwsMeasurements - 1) -
    timestampsFws(0);
  const int measPerSecFws = numFwsMeasurements / elapsedTimeFws;
  const int measPerSecDesiredFws = 5;
  int numSegmentsFws;
  if (measPerSecFws > measPerSecDesiredFws)
    numSegmentsFws = measPerSecDesiredFws * elapsedTimeFws;
  else
    numSegmentsFws = numFwsMeasurements;
  std::cout << "elapsedTimeFws: " << elapsedTimeFws << std::endl;
  std::cout << "measPerSecFws: " << measPerSecFws << std::endl;
  std::cout << "numFwsMeasurements: " << numFwsMeasurements << std::endl;
  std::cout << "numSegmentsFws: " << numSegmentsFws << std::endl;
  bsplineFws.initSplineSparse(timestampsFws, pointsFws, numSegmentsFws, 1e-6);

  // create B-Spline for rear wheel speeds
  std::cout << "Creating B-spline for rear wheel speeds..." << std::endl;
  const int orderRws = 4;
  BSpline bsplineRws(orderRws);
  const size_t numRwsMeasurements = canParser.getNumRearWheels();
  Eigen::VectorXd timestampsRws(numRwsMeasurements);
  Eigen::MatrixXd pointsRws(2, numRwsMeasurements);
  i = 0;
  for (auto it = canParser.cbeginRw(); it != canParser.cendRw(); ++it) {
    timestampsRws(i) = it->first;
    pointsRws.col(i) = Eigen::Vector2d(it->second.first, it->second.second);
    ++i;
  }
  const double elapsedTimeRws = timestampsRws(numRwsMeasurements - 1) -
    timestampsRws(0);
  const int measPerSecRws = numRwsMeasurements / elapsedTimeRws;
  const int measPerSecDesiredRws = 5;
  int numSegmentsRws;
  if (measPerSecRws > measPerSecDesiredRws)
    numSegmentsRws = measPerSecDesiredRws * elapsedTimeRws;
  else
    numSegmentsRws = numRwsMeasurements;
  std::cout << "elapsedTimeRws: " << elapsedTimeRws << std::endl;
  std::cout << "measPerSecRws: " << measPerSecRws << std::endl;
  std::cout << "numRwsMeasurements: " << numRwsMeasurements << std::endl;
  std::cout << "numSegmentsRws: " << numSegmentsRws << std::endl;
  bsplineRws.initSplineSparse(timestampsRws, pointsRws, numSegmentsRws, 1e-6);

  // create B-Spline for steering
  std::cout << "Creating B-spline for steering..." << std::endl;
  const int orderSt1 = 4;
  BSpline bsplineSt1(orderSt1);
  const size_t numSt1Measurements = canParser.getNumSteering1();
  Eigen::VectorXd timestampsSt1(numSt1Measurements);
  Eigen::MatrixXd pointsSt1(1, numSt1Measurements);
  i = 0;
  for (auto it = canParser.cbeginSt1(); it != canParser.cendSt1(); ++it) {
    timestampsSt1(i) = it->first;
    pointsSt1(0, i) = it->second;
    ++i;
  }
  const double elapsedTimeSt1 = timestampsSt1(numSt1Measurements - 1) -
    timestampsSt1(0);
  const int measPerSecSt1 = numSt1Measurements / elapsedTimeSt1;
  const int measPerSecDesiredSt1 = 5;
  int numSegmentsSt1;
  if (measPerSecSt1 > measPerSecDesiredSt1)
    numSegmentsSt1 = measPerSecDesiredSt1 * elapsedTimeSt1;
  else
    numSegmentsSt1 = numSt1Measurements;
  std::cout << "elapsedTimeSt1: " << elapsedTimeSt1 << std::endl;
  std::cout << "measPerSecSt1: " << measPerSecSt1 << std::endl;
  std::cout << "numSt1Measurements: " << numSt1Measurements << std::endl;
  std::cout << "numSegmentsSt1: " << numSegmentsSt1 << std::endl;
  bsplineSt1.initSplineSparse(timestampsSt1, pointsSt1, numSegmentsSt1, 1e-6);

  // intrisic odometry parameters found by optimization
  const double L = 2.698437452967974; // wheelbase [m]
  const double e_r = 0.744551686997985; // half-track rear [m]
  const double e_f = 0.7459111250452891; // half-track front [m]
  const double a0 = -0.8808883000114152; // steering coefficient
  const double a1 = 573.4542090947304587; // steering coefficient
  const double k_rl = 0.0028199619977176; // wheel coefficient
  const double k_rr = 0.0028260666435294; // wheel coefficient
  const double k_fl = 0.0028214039506398; // wheel coefficient
  const double k_fr = 0.0028223604206524; // wheel coefficient
  Odometry::VehicleParameters params = {0.285, 0.285, 0.28, 0.28, L,
    (e_r + e_f)};
  AckermanOdometry ackOdometry(params, Eigen::Vector3d(0, 0, yaw0));

  // time
  double tstart = std::max(timestampsFws(0), timestampsRws(0));
  const double t0 = std::max(tstart, timestampsSt1(0));
  double tend = std::min(timestampsFws(numFwsMeasurements - 1),
    timestampsRws(numRwsMeasurements - 1));
  tend = std::max(tend, timestampsSt1(numSt1Measurements - 1));
  const double T = 0.1;

  // integrate
  std::cout << "Integrating odometry..." << std::endl;
  double t = t0;
  while (t < tend) {
    Eigen::Vector2d fws = bsplineFws.eval(t);
    Eigen::Vector2d rws = bsplineRws.eval(t);
    Eigen::Matrix<double, 1, 1> st1 = bsplineSt1.eval(t);
    ackOdometry.updateWheelTranslationalVelocitiesSteering(k_rl * rws(1),
      k_rr * rws(0), k_fl * fws(1), k_fr * fws(0), (st1(0) - a0) / a1, T);
    t += T;
  }

  // output odometry
  std::ofstream odometryMATLABFile("integrated-odometry.txt");
  const std::vector<Eigen::Vector3d>& poses = ackOdometry.getPoseHistory();
  for (auto it = poses.cbegin(); it != poses.cend(); ++it)
    odometryMATLABFile << it->transpose() << std::endl;

  return 0;
}
