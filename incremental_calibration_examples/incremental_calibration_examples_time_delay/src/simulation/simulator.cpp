/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file simulator.cpp
    \brief This file estimates the calibration in simulation.
  */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#include <sm/BoostPropertyTree.hpp>

#include "aslam/calibration/time-delay/simulation/SimulationParams.h"
#include "aslam/calibration/time-delay/simulation/SimulationData.h"
#include "aslam/calibration/time-delay/simulation/simulationEngine.h"
#include "aslam/calibration/time-delay/algo/Calibrator.h"
#include "aslam/calibration/time-delay/algo/splinesToFile.h"

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
  SimulationParams params(PropertyTree(config, "time-delay/simulation/params"));

  std::cout << "Simulating..." << std::endl;

  // simulate
  SimulationData data;
  simulate(params, data);

  // write trajectory to file for visualization
  std::ofstream w_T_vFile("w_T_v.txt");
  w_T_vFile << std::fixed << std::setprecision(18);
  data.trajectory.w_T_vWrite(w_T_vFile);

  // write wheel velocities to file
  std::ofstream w_v_wwlFile("w_v_wwl.txt");
  data.w_v_wwlWrite(w_v_wwlFile);
  std::ofstream w_v_wwrFile("w_v_wwr.txt");
  data.w_v_wwrWrite(w_v_wwrFile);

  // write noise-free measurements to file
  std::ofstream rwDataFile("rwData.txt");
  data.rwDataWrite(rwDataFile);
  std::ofstream lwDataFile("lwData.txt");
  data.lwDataWrite(lwDataFile);
  std::ofstream poseDataFile("poseData.txt");
  data.poseDataWrite(poseDataFile);

  // write noisy measurements to file
  std::ofstream rwData_nFile("rwData_n.txt");
  data.rwData_nWrite(rwData_nFile);
  std::ofstream lwData_nFile("lwData_n.txt");
  data.lwData_nWrite(lwData_nFile);
  std::ofstream poseData_nFile("poseData_n.txt");
  data.poseData_nWrite(poseData_nFile);

  std::cout << "Optimizing..." << std::endl;
  Calibrator calibrator(PropertyTree(config, "time-delay/calibrator"));
  for (auto it = data.rwData_n.cbegin(); it != data.rwData_n.cend(); ++it) {
    auto rw = data.rwData_n.at(std::distance(data.rwData_n.cbegin(), it));
    calibrator.addRightWheelMeasurement(rw.second, rw.first);
    auto lw = data.lwData_n.at(std::distance(data.rwData_n.cbegin(), it));
    calibrator.addLeftWheelMeasurement(lw.second, lw.first);
    auto pose = data.poseData_n.at(std::distance(data.rwData_n.cbegin(), it));
    calibrator.addPoseMeasurement(pose.second, pose.first);
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

  std::ofstream calibHistFile("calibHist.txt");
  auto calibHist = calibrator.getOdometryVariablesHistory();
  calibHistFile << std::fixed << std::setprecision(18);
  std::for_each(calibHist.cbegin(), calibHist.cend(), [&](decltype(
    *calibHist.cbegin()) x) {calibHistFile << x.transpose() << std::endl;});

  return 0;
}
