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

/** \file CarCalibratorTest.cpp
    \brief This file tests the CarCalibrator class.
  */

#include <gtest/gtest.h>

#include <aslam/calibration/exceptions/InvalidOperationException.h>

#include "aslam/calibration/car/CarCalibrator.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testCarCalibrator) {
  CarCalibrator::CalibrationDesignVariables dv;
  CarCalibrator::IncrementalEstimatorSP estimator;
  CarCalibrator calibrator(estimator, dv);
  calibrator.addMeasurement(CarCalibrator::ApplanixNavigationMeasurement(), 10);
  ASSERT_THROW(calibrator.addMeasurement(
    CarCalibrator::ApplanixNavigationMeasurement(), 5),
    InvalidOperationException);
}
