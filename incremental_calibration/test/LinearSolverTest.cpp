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

/** \file VectorDesignVariableTest.cpp
    \brief This file tests the VectorDesignVariable class.
  */

#include <cstddef>
#include <iostream>
#include <iomanip>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cholmod.h>
#include <SuiteSparseQR.hpp>

#include "aslam/calibration/core/LinearSolver.h"
#include "aslam/calibration/algorithms/linalg.h"
#include "aslam/calibration/base/Timestamp.h"

TEST(AslamCalibrationTestSuite, testLinearSolver) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 30);
  const Eigen::VectorXd x = Eigen::VectorXd::Random(30);
  const Eigen::VectorXd b = A * x;
  cholmod_common cholmod;
  cholmod_l_start(&cholmod);
  cholmod_sparse* A_CS = aslam::calibration::eigenDenseToCholmodSparseCopy(A,
    &cholmod);
  cholmod_dense b_CD;
  aslam::calibration::eigenDenseToCholmodDenseView(b, &b_CD);
  Eigen::VectorXd x_est;
//  std::cout << "SVD-SPQR solver" << std::endl;
  double before, after, error;
  aslam::calibration::LinearSolver linearSolver;
  for (std::ptrdiff_t i = 1; i < x.size(); ++i) {
    before = aslam::calibration::Timestamp::now();
    linearSolver.solve(A_CS, &b_CD, i, x_est);
    after = aslam::calibration::Timestamp::now();
    error = (b - A * x_est).norm();
//    std::cout << std::fixed << std::setprecision(18) << "noscale: " << "error: "
//      << error << " est_diff: " << (x - x_est).norm() << " time: "
//      << after - before << std::endl;
    ASSERT_NEAR(error, 0, 1e-9);
    linearSolver.getOptions().columnScaling = true;
    before = aslam::calibration::Timestamp::now();
    linearSolver.solve(A_CS, &b_CD, i, x_est);
    after = aslam::calibration::Timestamp::now();
    error = (b - A * x_est).norm();
//    std::cout << std::fixed << std::setprecision(18) << "onscale: " << "error: "
//      << error << " est_diff: " << (x - x_est).norm() << " time: "
//      << after - before << std::endl;
    linearSolver.getOptions().columnScaling = false;
    ASSERT_NEAR(error, 0, 1e-9);
  }
//  std::cout << "SVD solver" << std::endl;
  before = aslam::calibration::Timestamp::now();
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  x_est = svd.solve(b);
  after = aslam::calibration::Timestamp::now();
  error = (b - A * x_est).norm();
//  std::cout << std::fixed << std::setprecision(18) << "error: " << error
//    << " est_diff: " << (x - x_est).norm() << " time: " << after - before
//    << std::endl;
//  std::cout << "SPQR solver" << std::endl;
  before = aslam::calibration::Timestamp::now();
  SuiteSparseQR_factorization<double>* factor = SuiteSparseQR_factorize<double>(
    SPQR_ORDERING_BEST, SPQR_DEFAULT_TOL, A_CS, &cholmod);
  cholmod_dense* Qtb = SuiteSparseQR_qmult<double>(SPQR_QTX, factor, &b_CD,
    &cholmod);
  cholmod_dense* x_est_cd = SuiteSparseQR_solve<double>(SPQR_RETX_EQUALS_B,
    factor, Qtb, &cholmod);
  cholmod_l_free_dense(&Qtb, &cholmod);
  aslam::calibration::cholmodDenseToEigenDenseCopy(x_est_cd, x_est);
  cholmod_l_free_dense(&x_est_cd, &cholmod);
  SuiteSparseQR_free(&factor, &cholmod);
  after = aslam::calibration::Timestamp::now();
  error = (b - A * x_est).norm();
//  std::cout << std::fixed << std::setprecision(18) << "error: " << error
//    << " est_diff: " << (x - x_est).norm() << " time: " << after - before
//    << std::endl;
  cholmod_l_free_sparse(&A_CS, &cholmod);
  cholmod_l_finish(&cholmod);
}
