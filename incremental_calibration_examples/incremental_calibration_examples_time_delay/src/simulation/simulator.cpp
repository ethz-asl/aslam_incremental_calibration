/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

#include <sm/BoostPropertyTree.hpp>

#include "aslam/calibration/time-delay/simulation/SimulationParams.h"
#include "aslam/calibration/time-delay/simulation/SimulationData.h"
#include "aslam/calibration/time-delay/simulation/simulationEngine.h"

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

  return 0;
}
