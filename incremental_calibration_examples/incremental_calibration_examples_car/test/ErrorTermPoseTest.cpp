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

#include <gtest/gtest.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/statistics/NormalDistribution.h"

TEST(AslamCalibrationTestSuite, testErrorTermPose) {

  // estimated pose, angles: yaw-pitch-roll
  const aslam::calibration::ErrorTermPose::Input xe(
    (aslam::calibration::ErrorTermPose::Input() << 100, 200, 300, M_PI / 2.0,
      M_PI / 4.0, M_PI / 8.0).finished());

  // covariance matrix of the measurement
  aslam::calibration::ErrorTermPose::Covariance Q =
    aslam::calibration::ErrorTermPose::Covariance::Zero();
  Q(0, 0) = 1e-4;
  Q(1, 1) = 1e-4;
  Q(2, 2) = 1e-4;
  Q(3, 3) = 4e-4;
  Q(4, 4) = 4e-4;
  Q(5, 5) = 4e-4;

  // measured pose
  const aslam::calibration::ErrorTermPose::Input xm(
    aslam::calibration::NormalDistribution<6>(xe, Q).getSample());

  // encapsulate the estimated pose into a transformation expression
  boost::shared_ptr<aslam::backend::EuclideanPoint> translation(
    new aslam::backend::EuclideanPoint(xe.head<3>()));
  sm::kinematics::EulerAnglesYawPitchRoll conv;
  boost::shared_ptr<aslam::backend::RotationQuaternion> rotation(
    new aslam::backend::RotationQuaternion(
    conv.parametersToRotationMatrix(xe.tail<3>())));
  aslam::backend::TransformationExpression T(
    aslam::backend::RotationExpression(rotation.get()),
    aslam::backend::EuclideanExpression(translation.get()));

  // build the error term
  aslam::calibration::ErrorTermPose e1(T, xm, Q);

  // test the error term
  try {
    aslam::backend::ErrorTermTestHarness<6> harness(&e1);
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
