/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
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

/** \file simulator.cpp
    \brief This file estimates the calibration in simulation.
  */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <string>

#include <sm/BoostPropertyTree.hpp>

#include "aslam/calibration/egomotion/simulation/SimulationParams.h"
#include "aslam/calibration/egomotion/simulation/SimulationData.h"
#include "aslam/calibration/egomotion/simulation/simulationEngine.h"
#include "aslam/calibration/egomotion/algo/Calibrator.h"
#include "aslam/calibration/egomotion/algo/splinesToFile.h"

using namespace aslam::calibration;
using namespace sm;

int main(int argc, char** argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <conf_file>" << std::endl;
    return -1;
  }

  // load config file
  BoostPropertyTree config;
  config.loadXml(argv[1]);

  // load simulation parameters
  SimulationParams params(PropertyTree(config, "egomotion/simulation/params"));

  std::cout << "Simulating..." << std::endl;

  // simulate
  SimulationData data;
  simulate(params, data);

  // write trajectory to file for visualization
  std::ofstream w_T_vFile("w_T_v.txt");
  w_T_vFile << std::fixed << std::setprecision(18);
  writeSplines(data.trajectory.translationSpline,
    data.trajectory.rotationSpline, params.dt, w_T_vFile);

  // write noise-free measurements to file
  for (const auto& motionData : data.motionData) {
    std::stringstream filenameStream;
    filenameStream << "motionData_" << motionData.first << ".txt";
    std::ofstream motionDataFile(filenameStream.str());
    motionDataFile << std::fixed << std::setprecision(18);
    std::for_each(motionData.second.cbegin(), motionData.second.cend(),
      [&](decltype(*motionData.second.cbegin()) x){
      motionDataFile << x.second.motion.t().transpose() << " "
      << x.second.motion.q().transpose() << std::endl;});
  }

  // write noisy measurements to file
  for (const auto& motionData : data.motionDataNoisy) {
    std::stringstream filenameStream;
    filenameStream << "motionDataNoisy_" << motionData.first << ".txt";
    std::ofstream motionDataFile(filenameStream.str());
    motionDataFile << std::fixed << std::setprecision(18);
    std::for_each(motionData.second.cbegin(), motionData.second.cend(),
      [&](decltype(*motionData.second.cbegin()) x){
      motionDataFile << x.second.motion.t().transpose() << " "
      << x.second.motion.q().transpose() << std::endl;});
  }

  std::cout << "Optimizing..." << std::endl;
  Calibrator calibrator(PropertyTree(config, "egomotion/calibrator"));

  const auto referenceSensor =
    config.getInt("egomotion/calibrator/referenceSensor");
  for (auto it = data.motionData.at(referenceSensor).cbegin();
      it != data.motionData.at(referenceSensor).cend(); ++it) {
    for (const auto& motionData : data.motionData) {
      auto motion = motionData.second.at(
        std::distance(data.motionData.at(referenceSensor).cbegin(), it));
      calibrator.addMotionMeasurement(motion.second, motion.first,
        motionData.first);
    }
  }

  if (calibrator.unprocessedMeasurements())
    calibrator.addMeasurements();

  std::cout << "Output results to file..." << std::endl;

  std::ofstream w_T_v_estFile("w_T_v_est.txt");
  w_T_v_estFile << std::fixed << std::setprecision(18);
  writeSplines(calibrator.getEstimator(), params.dt, w_T_v_estFile);

  std::ofstream infoGainHistFile("infoGainHist.txt");
  auto infoGainHist = calibrator.getInformationGainHistory();
  infoGainHistFile << std::fixed << std::setprecision(18);
  std::for_each(infoGainHist.cbegin(), infoGainHist.cend(), [&](decltype(
    *infoGainHist.cbegin()) x) {infoGainHistFile << x << std::endl;});

  for (const auto& calib : calibrator.getDesignVariablesHistory()) {
    std::stringstream filenameStream;
    filenameStream << "calibHist_" << calib.first << ".txt";
    std::ofstream calibHistFile(filenameStream.str());
    calibHistFile << std::fixed << std::setprecision(18);
    std::for_each(calib.second.cbegin(), calib.second.cend(), [&](decltype(
      *calib.second.cbegin()) x) {calibHistFile << x.transpose()
      << std::endl;});
  }

  return 0;
}
