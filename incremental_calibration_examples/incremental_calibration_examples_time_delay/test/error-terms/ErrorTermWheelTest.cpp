/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermWheelTest.cpp
    \brief This file tests the ErrorTermWheel class.
  */

#include <cmath>

#include <boost/make_shared.hpp>

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/ScalarExpression.hpp>

#include "aslam/calibration/time-delay/error-terms/ErrorTermWheel.h"

using namespace aslam::backend;
using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testErrorTermWheel) {

  // linear velocity
  const Eigen::Matrix<double, 3, 1> v(1.5, 1.7, 2.8);
  auto v_dv = boost::make_shared<EuclideanPoint>(v);
  EuclideanExpression v_exp(v_dv);

  // scaling factor
  auto k_vd = boost::make_shared<Scalar>(1.6);
  ScalarExpression k_exp(k_vd);

  // error term rear
  ErrorTermWheel e_r(v_exp, k_exp, 1.5, Eigen::Matrix3d::Identity());

  // error term front
  ErrorTermWheel e_f(v_exp, k_exp, 1.5, Eigen::Matrix3d::Identity(), true);

  // test the error terms
  try {
    ErrorTermTestHarness<3> harness(&e_r);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }
  try {
    ErrorTermTestHarness<3> harness(&e_f);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // test accessors
  e_r.setCovariance(Eigen::Matrix3d::Identity());
  ASSERT_EQ(e_r.getCovariance(), Eigen::Matrix3d::Identity());
  e_r.setMeasurement(10);
  ASSERT_EQ(e_r.getMeasurement(), 10);
  ASSERT_TRUE(e_f.getFrontEnabled());
  ASSERT_FALSE(e_r.getFrontEnabled());
  e_r.setFrontEnabled();
  ASSERT_TRUE(e_r.getFrontEnabled());
}
