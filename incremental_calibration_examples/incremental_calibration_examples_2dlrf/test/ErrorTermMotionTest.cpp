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

/** \file ErrorTermMotionTest.cpp
    \brief This file tests the ErrorTermMotion class.
  */

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/2dlrf/ErrorTermMotion.h"

TEST(AslamCalibrationTestSuite, testErrorTermMotion) {
  // state at time k-1
  aslam::calibration::VectorDesignVariable<3> xkm1(
    aslam::calibration::VectorDesignVariable<3>::Container(1.0, 1.0, 0.78));

  // timestep
  const double T = 0.1;

  // input at time k
  const aslam::calibration::ErrorTermMotion::Input uk(
    aslam::calibration::ErrorTermMotion::Input(1.0, 0.0, 0));

  // matrix for motion model
  Eigen::Matrix<double, 3, 3> B = Eigen::Matrix<double, 3, 3>::Identity();
  B(0, 0) = cos((xkm1.getValue())(2));
  B(0, 1) = -sin((xkm1.getValue())(2));
  B(1, 0) = sin((xkm1.getValue())(2));
  B(1, 1) = cos((xkm1.getValue())(2));

  // covariance matrix
  aslam::calibration::ErrorTermMotion::Covariance Q =
    aslam::calibration::ErrorTermMotion::Covariance::Zero();
  Q(0, 0) = 0.00044;
  Q(1, 1) = 1e-6;
  Q(2, 2) = 0.00082;

  // state at time k
  aslam::calibration::VectorDesignVariable<3> xk(xkm1.getValue() + T * B *
    (uk + aslam::calibration::NormalDistribution<3>(
    aslam::calibration::NormalDistribution<3>::Mean::Zero(), Q).getSample()));

  // error term for motion model
  aslam::calibration::ErrorTermMotion e1(&xkm1, &xk, T, uk, Q);

  // test the error term
  try {
    aslam::backend::ErrorTermTestHarness<3> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // accessors test
  e1.setTimestep(T);
  e1.setInput(uk);
  e1.setCovariance(Q);
  ASSERT_EQ(e1.getTimestep(), T);
  ASSERT_EQ(e1.getInput(), uk);
  ASSERT_EQ(e1.getCovariance(), Q);
}
