/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
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
