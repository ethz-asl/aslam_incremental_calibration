/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermObservationTest.cpp
    \brief This file tests the ErrorTermObservation class.
  */

#include <gtest/gtest.h>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/2dlrf/ErrorTermObservation.h"

TEST(AslamCalibrationTestSuite, testErrorTermObservation) {
  // state at time k
  aslam::calibration::VectorDesignVariable<3> xk(
    aslam::calibration::VectorDesignVariable<3>::Container(1.0, 1.0, 0.78));

  // one landmark
  aslam::calibration::VectorDesignVariable<2> xl(
    aslam::calibration::VectorDesignVariable<2>::Container(5.0, 5.0));

  // calibration parameters
  aslam::calibration::VectorDesignVariable<3> Theta(
    aslam::calibration::VectorDesignVariable<3>::Container(0.219, 0.1, 0.78));

  // covariance matrix
  aslam::calibration::ErrorTermObservation::Covariance R =
    aslam::calibration::ErrorTermObservation::Covariance::Zero();
  R(0, 0) = 0.00090;
  R(1, 1) = 0.00067;

  // some computations
  const double ct = cos((xk.getValue())(2));
  const double st = sin((xk.getValue())(2));
  const double dxct = (Theta.getValue())(0) * ct;
  const double dxst = (Theta.getValue())(0) * st;
  const double dyct = (Theta.getValue())(1) * ct;
  const double dyst = (Theta.getValue())(1) * st;
  const double aa = (xl.getValue())(0) - (xk.getValue())(0) - dxct + dyst;
  const double bb = (xl.getValue())(1) - (xk.getValue())(1) - dxst - dyct;
  const double temp1 = aa * aa + bb * bb;
  const double temp2 = sqrt(temp1);

  // range
  const double r = temp2 +
    aslam::calibration::NormalDistribution<1>(0, R(0, 0)).getSample();

  // bearing
  const double b = atan2(bb, aa) - (xk.getValue())(2) - (Theta.getValue())(2) +
    aslam::calibration::NormalDistribution<1>(0, R(1, 1)).getSample();

  // error term for observation model
  aslam::calibration::ErrorTermObservation e1(&xk, &xl, &Theta, r, b, R);

  // test the error term
  try {
    aslam::backend::ErrorTermTestHarness<2> harness(&e1);
    harness.testAll();
  }
  catch (const std::exception& e) {
    FAIL() << e.what();
  }

  // accessors test
  e1.setRange(r);
  e1.setBearing(b);
  e1.setCovariance(R);
  ASSERT_EQ(e1.getRange(), r);
  ASSERT_EQ(e1.getBearing(), b);
  ASSERT_EQ(e1.getCovariance(), R);
}
