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

/** \file ErrorTermVelocitiesTest.cpp
    \brief This file tests the ErrorTermVelocities class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/car/error-terms/ErrorTermVelocities.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermVelocities) {

  // estimated velocities
  const ErrorTermVelocities::Input v_e((ErrorTermVelocities::Input() <<
    5, 6, 7).finished());
  const ErrorTermVelocities::Input om_e((ErrorTermVelocities::Input() <<
    M_PI / 2.0, M_PI / 4.0, M_PI / 8.0).finished());

  // covariance matrix
  ErrorTermVelocities::Covariance Q = ErrorTermVelocities::Covariance::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;

  // measured velocities
  const ErrorTermVelocities::Input v_m(
    NormalDistribution<3>(v_e, Q).getSample());
  const ErrorTermVelocities::Input om_m(
    NormalDistribution<3>(om_e, Q).getSample());

  // encapsulate the estimated velocities into Euclidean expression
  auto r_v_mr = boost::make_shared<EuclideanPoint>(v_e);
  EuclideanExpression r_v_mr_e(r_v_mr);
  auto r_om_mr = boost::make_shared<EuclideanPoint>(om_e);
  EuclideanExpression r_om_mr_e(r_om_mr);

  // build the error term
  ErrorTermVelocities e1(r_v_mr_e, r_om_mr_e, v_m, om_m, Q, Q);

  // test the error term
  try {
    ErrorTermTestHarness<6> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
}
