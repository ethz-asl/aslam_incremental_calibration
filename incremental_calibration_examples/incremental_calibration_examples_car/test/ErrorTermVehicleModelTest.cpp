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

/** \file ErrorTermVehicleModelTest.cpp
    \brief This file tests the ErrorTermVehicleModelTest class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>

#include "aslam/calibration/car/ErrorTermVehicleModel.h"

TEST(AslamCalibrationTestSuite, testErrorTermVehicleModel) {

  // linear velocity
  const Eigen::Matrix<double, 3, 1> v(1.5, 0, 0);
  auto v_en = boost::make_shared<aslam::backend::EuclideanPoint>(v);
  aslam::backend::EuclideanExpression v_e(v_en);

  // angular velocity
  const Eigen::Matrix<double, 3, 1> om(0, 0, M_PI / 8.0);
  auto om_en = boost::make_shared<aslam::backend::EuclideanPoint>(om);
  aslam::backend::EuclideanExpression om_e(om_en);

  // covariance matrix of the pseudo-measurement
  aslam::calibration::ErrorTermVehicleModel::Covariance Q =
    aslam::calibration::ErrorTermVehicleModel::Covariance::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;
  Q(3, 3) = 1e-4;
  Q(4, 4) = 1e-4;

  // build the error term
  aslam::calibration::ErrorTermVehicleModel e(v_e, om_e, Q);

  // test the error term
  try {
    aslam::backend::ErrorTermTestHarness<4> harness(&e);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // accessors test
  e.setCovariance(Q);
  ASSERT_EQ(e.getCovariance(), Q);
}
