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

/** \file ErrorTermPoseTest.cpp
    \brief This file tests the ErrorTermPose class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/car/error-terms/ErrorTermPose.h"

using namespace aslam::backend;
using namespace aslam::calibration;
using namespace sm::kinematics;

TEST(AslamCalibrationTestSuite, testErrorTermPose) {

  // estimated pose, angles: yaw-pitch-roll
  const ErrorTermPose::Input xe((ErrorTermPose::Input() << 100, 200, 300,
    M_PI / 2.0, M_PI / 4.0, M_PI / 8.0).finished());

  // covariance matrix of the measurement
  ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;
  Q(3, 3) = 4e-4;
  Q(4, 4) = 4e-4;
  Q(5, 5) = 4e-4;

  // measured pose
  const ErrorTermPose::Input xm(NormalDistribution<6>(xe, Q).getSample());

  // encapsulate the estimated pose into a transformation expression
  auto translation = boost::make_shared<EuclideanPoint>(xe.head<3>());
  EulerAnglesYawPitchRoll conv;
  auto rotation = boost::make_shared<RotationQuaternion>(
    conv.parametersToRotationMatrix(xe.tail<3>()));
  TransformationExpression T(RotationExpression(rotation.get()),
    EuclideanExpression(translation.get()));

  // build the error term
  ErrorTermPose e1(T, xm, Q);

  // test the error term
  try {
    ErrorTermTestHarness<6> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // accessors test
  e1.setInput(xm);
  e1.setCovariance(Q);
  ASSERT_EQ(e1.getInput(), xm);
  ASSERT_EQ(e1.getCovariance(), Q);
}
