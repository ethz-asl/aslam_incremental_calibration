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

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/exceptions/InvalidOperationException.h>

#include "aslam/calibration/car/CarCalibrator.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testCarCalibrator) {
  CarCalibrator::CalibrationDesignVariables dv;
  dv.intrinsicCANDesignVariable =
    boost::make_shared<VectorDesignVariable<11> >();
  dv.intrinsicDMIDesignVariable =
    boost::make_shared<VectorDesignVariable<1> >();
  dv.extrinsicOdometryTranslationDesignVariable =
    boost::make_shared<EuclideanPoint>(Eigen::Vector3d(0, 0, -0.785));
  dv.extrinsicOdometryRotationDesignVariable =
    boost::make_shared<RotationQuaternion>(Eigen::Matrix3d());
  auto estimator = boost::make_shared<IncrementalEstimator>(1);
  CarCalibrator calibrator(estimator, dv);
  calibrator.addMeasurement(CarCalibrator::ApplanixNavigationMeasurement(), 10);
  ASSERT_THROW(calibrator.addMeasurement(
    CarCalibrator::ApplanixNavigationMeasurement(), 5),
    InvalidOperationException);
}
