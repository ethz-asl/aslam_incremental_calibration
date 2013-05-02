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

/** \file odometryBSpline.cpp
    \brief This file processes the odometry.
  */

#include <iostream>
#include <fstream>

#include <bsplines/BSpline.hpp>

#include "aslam/calibration/car/CANBinaryParser.h"

using namespace aslam::calibration;
using namespace bsplines;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <LogFilename>" << std::endl;
    return -1;
  }

  // parse the CAN log file
  std::cout << "Parsing CAN data..." << std::endl;
  CANBinaryParser canParser(argv[1]);
  canParser.parse();

  // output to MATLAB readable file
  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream canRawFwMATLABFile("can-raw-fws.txt");
  canParser.writeFwMATLAB(canRawFwMATLABFile);
  std::ofstream canRawRwMATLABFile("can-raw-rws.txt");
  canParser.writeRwMATLAB(canRawRwMATLABFile);
  std::ofstream canRawStMATLABFile("can-raw-st.txt");
  canParser.writeStMATLAB(canRawStMATLABFile);

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
  const int orderSt = 4;
  BSpline bsplineSt(orderSt);
  const size_t numStMeasurements = canParser.getNumSteering();
  Eigen::VectorXd timestampsSt(numStMeasurements);
  Eigen::MatrixXd pointsSt(1, numStMeasurements);
  i = 0;
  for (auto it = canParser.cbeginSt(); it != canParser.cendSt(); ++it) {
    timestampsSt(i) = it->first;
    pointsSt(0, i) = it->second;
    ++i;
  }
  const double elapsedTimeSt = timestampsSt(numStMeasurements - 1) -
    timestampsSt(0);
  const int measPerSecSt = numStMeasurements / elapsedTimeSt;
  const int measPerSecDesiredSt = 5;
  int numSegmentsSt;
  if (measPerSecSt > measPerSecDesiredSt)
    numSegmentsSt = measPerSecDesiredSt * elapsedTimeSt;
  else
    numSegmentsSt = numStMeasurements;
  std::cout << "elapsedTimeSt: " << elapsedTimeSt << std::endl;
  std::cout << "measPerSecSt: " << measPerSecSt << std::endl;
  std::cout << "numStMeasurements: " << numStMeasurements << std::endl;
  std::cout << "numSegmentsSt: " << numSegmentsSt << std::endl;
  bsplineSt.initSplineSparse(timestampsSt, pointsSt, numSegmentsSt, 1e-6);

  // output B-spline data
  std::cout << "Outputting spline data to MATLAB..." << std::endl;
  std::ofstream canSplineFwMATLABFile("can-spline-fws.txt");
  for (size_t i = 0; i < numFwsMeasurements; ++i)
    canSplineFwMATLABFile << std::fixed << std::setprecision(16)
      << timestampsFws(i) << " "
      << bsplineFws.eval(timestampsFws(i)).transpose() << std::endl;
  std::ofstream canSplineRwMATLABFile("can-spline-rws.txt");
  for (size_t i = 0; i < numRwsMeasurements; ++i)
    canSplineRwMATLABFile << std::fixed << std::setprecision(16)
      << timestampsRws(i) << " "
      << bsplineRws.eval(timestampsRws(i)).transpose() << std::endl;
  std::ofstream canSplineStMATLABFile("can-spline-st.txt");
  for (size_t i = 0; i < numStMeasurements; ++i)
    canSplineStMATLABFile << std::fixed << std::setprecision(16)
      << timestampsSt(i) << " "
      << bsplineSt.eval(timestampsSt(i)).transpose() << std::endl;
  return 0;
}
