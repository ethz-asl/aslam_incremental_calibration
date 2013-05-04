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

/** \file odometryDump.cpp
    \brief This file dumps the odometry.
  */

#include <iostream>
#include <fstream>

#include "aslam/calibration/car/CANBinaryParser.h"

using namespace aslam::calibration;

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
  std::ofstream canRawSt1MATLABFile("can-raw-st1.txt");
  canParser.writeSt1MATLAB(canRawSt1MATLABFile);
  std::ofstream canRawSt2MATLABFile("can-raw-st2.txt");
  canParser.writeSt2MATLAB(canRawSt2MATLABFile);
  std::ofstream canRawAcc1MATLABFile("can-raw-acc1.txt");
  canParser.writeAcc1MATLAB(canRawAcc1MATLABFile);
  std::ofstream canRawAcc2MATLABFile("can-raw-acc2.txt");
  canParser.writeAcc2MATLAB(canRawAcc2MATLABFile);
  std::ofstream canRawSp1MATLABFile("can-raw-sp1.txt");
  canParser.writeSp1MATLAB(canRawSp1MATLABFile);
  std::ofstream canRawSp2MATLABFile("can-raw-sp2.txt");
  canParser.writeSp2MATLAB(canRawSp2MATLABFile);
  std::ofstream canRawSp3MATLABFile("can-raw-sp3.txt");
  canParser.writeSp3MATLAB(canRawSp3MATLABFile);
  return 0;
}
