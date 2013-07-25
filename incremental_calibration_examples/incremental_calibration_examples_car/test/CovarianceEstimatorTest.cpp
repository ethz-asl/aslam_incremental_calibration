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

/** \file CovarianceEstimatorTest.cpp
    \brief This file tests the CovarianceEstimator class.
  */

#include <vector>
#include <iostream>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <sm/eigen/gtest.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/car/CovarianceEstimator.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testCovarianceEstimator) {
  CovarianceEstimator<2> estimator1;
  std::vector<Eigen::Vector2d> x1;
  NormalDistribution<2>().getSamples(x1, 100000);
  for (auto it = x1.cbegin(); it != x1.cend(); ++it)
    estimator1.addMeasurement(*it);
  ASSERT_NEAR(estimator1.getEstChiSquaredMean(), estimator1.getChiSquaredMean(),
    1e-1);
  ASSERT_NEAR(estimator1.getEstChiSquaredVariance(),
    estimator1.getChiSquaredVariance(), 1e-1);
  sm::eigen::assertNear(estimator1.getCovariance(), Eigen::Matrix2d::Identity(),
    1e-1, SM_SOURCE_FILE_POS, "Bad covariance estimate");
  sm::eigen::assertNear(estimator1.getMean(), Eigen::Vector2d::Zero(),
    1e-1, SM_SOURCE_FILE_POS, "Bad mean estimate");
  CovarianceEstimator<1> estimator2;
  std::vector<double> x2;
  NormalDistribution<1>().getSamples(x2, 100000);
  for (auto it = x2.cbegin(); it != x2.cend(); ++it)
    estimator2.addMeasurement(
      (Eigen::Matrix<double, 1, 1>() << *it).finished());
  ASSERT_NEAR(estimator2.getEstChiSquaredMean(), estimator2.getChiSquaredMean(),
    1e-1);
  ASSERT_NEAR(estimator2.getEstChiSquaredVariance(),
    estimator2.getChiSquaredVariance(), 1e-1);
  sm::eigen::assertNear(estimator2.getCovariance(),
    Eigen::Matrix<double, 1, 1>::Identity(),
    1e-1, SM_SOURCE_FILE_POS, "Bad covariance estimate");
  sm::eigen::assertNear(estimator2.getMean(),
    Eigen::Matrix<double, 1, 1>::Zero(),
    1e-1, SM_SOURCE_FILE_POS, "Bad mean estimate");
  CovarianceEstimator<3> estimator3;
  std::vector<Eigen::Vector3d> x3;
  NormalDistribution<3>().getSamples(x3, 100000);
  for (auto it = x3.cbegin(); it != x3.cend(); ++it)
    estimator3.addMeasurement(*it);
  ASSERT_NEAR(estimator3.getEstChiSquaredMean(), estimator3.getChiSquaredMean(),
    1e-1);
  ASSERT_NEAR(estimator3.getEstChiSquaredVariance(),
    estimator3.getChiSquaredVariance(), 1e-1);
  sm::eigen::assertNear(estimator3.getCovariance(), Eigen::Matrix3d::Identity(),
    1e-1, SM_SOURCE_FILE_POS, "Bad covariance estimate");
  sm::eigen::assertNear(estimator3.getMean(), Eigen::Vector3d::Zero(),
    1e-1, SM_SOURCE_FILE_POS, "Bad mean estimate");
}
