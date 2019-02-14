/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
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

#include "aslam/calibration/time-delay/error-terms/ErrorTermPose.h"

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
