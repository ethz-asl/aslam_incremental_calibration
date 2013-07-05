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

#include "aslam/calibration/car/CovarianceEstimator.h"
#include <aslam/calibration/statistics/NormalDistribution.h>

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testCovarianceEstimator) {
  CovarianceEstimator<2> estimator1;
  std::vector<Eigen::Vector2d> x1;
  NormalDistribution<2>().getSamples(x1, 100000);
  for (auto it = x1.cbegin(); it != x1.cend(); ++it)
    estimator1.addMeasurement(*it);
  std::cout << "Mean: " << std::endl << estimator1.getMean() << std::endl
    << std::endl;
  std::cout << "Covariance: " << std::endl << estimator1.getCovariance()
    << std::endl << std::endl;
  std::cout << "Estimated chi-square mean: " << std::endl
    << estimator1.getEstChiSquaredMean() << std::endl << std::endl;
  std::cout << "Estimated chi-square variance: " << std::endl
    << estimator1.getEstChiSquaredVariance() << std::endl << std::endl;
  std::cout << "Estimated chi-square mode: " << std::endl
    << estimator1.getEstChiSquaredMode() << std::endl << std::endl;
  std::cout << "True chi-square mean: " << std::endl
    << estimator1.getChiSquaredMean() << std::endl << std::endl;
  std::cout << "True chi-square variance: " << std::endl
    << estimator1.getChiSquaredVariance() << std::endl << std::endl;
  std::cout << "True chi-square mode: " << std::endl
    << estimator1.getChiSquaredMode() << std::endl << std::endl;
  CovarianceEstimator<1> estimator2;
  std::vector<double> x2;
  NormalDistribution<1>().getSamples(x2, 100000);
  for (auto it = x2.cbegin(); it != x2.cend(); ++it)
    estimator2.addMeasurement(
      (Eigen::Matrix<double, 1, 1>() << *it).finished());
  std::cout << "Mean: " << std::endl << estimator2.getMean() << std::endl
    << std::endl;
  std::cout << "Covariance: " << std::endl << estimator2.getCovariance()
    << std::endl << std::endl;
  std::cout << "Estimated chi-square mean: " << std::endl
    << estimator2.getEstChiSquaredMean() << std::endl << std::endl;
  std::cout << "Estimated chi-square variance: " << std::endl
    << estimator2.getEstChiSquaredVariance() << std::endl << std::endl;
  std::cout << "Estimated chi-square mode: " << std::endl
    << estimator2.getEstChiSquaredMode() << std::endl << std::endl;
  std::cout << "True chi-square mean: " << std::endl
    << estimator2.getChiSquaredMean() << std::endl << std::endl;
  std::cout << "True chi-square variance: " << std::endl
    << estimator2.getChiSquaredVariance() << std::endl << std::endl;
  CovarianceEstimator<3> estimator3;
  std::vector<Eigen::Vector3d> x3;
  NormalDistribution<3>().getSamples(x3, 100000);
  for (auto it = x3.cbegin(); it != x3.cend(); ++it)
    estimator3.addMeasurement(*it);
  std::cout << "Mean: " << std::endl << estimator3.getMean() << std::endl
    << std::endl;
  std::cout << "Covariance: " << std::endl << estimator3.getCovariance()
    << std::endl << std::endl;
  std::cout << "Estimated chi-square mean: " << std::endl
    << estimator3.getEstChiSquaredMean() << std::endl << std::endl;
  std::cout << "Estimated chi-square variance: " << std::endl
    << estimator3.getEstChiSquaredVariance() << std::endl << std::endl;
  std::cout << "Estimated chi-square mode: " << std::endl
    << estimator3.getEstChiSquaredMode() << std::endl << std::endl;
  std::cout << "True chi-square mean: " << std::endl
    << estimator3.getChiSquaredMean() << std::endl << std::endl;
  std::cout << "True chi-square variance: " << std::endl
    << estimator3.getChiSquaredVariance() << std::endl << std::endl;
  std::cout << "True chi-square mode: " << std::endl
    << estimator3.getChiSquaredMode() << std::endl << std::endl;
}
