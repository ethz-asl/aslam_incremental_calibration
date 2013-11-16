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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cholmod.h>

#include <aslam/backend/CompressedColumnMatrix.hpp>

#include "aslam/calibration/core/LinearSolver.h"
#include "aslam/calibration/algorithms/linalg.h"

TEST(AslamCalibrationTestSuite, testLinearSolver) {
  aslam::calibration::LinearSolver linearSolver;
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 30);
  const Eigen::VectorXd x = Eigen::VectorXd::Random(30);
  const Eigen::VectorXd b = A * x;
  aslam::backend::CompressedColumnMatrix<std::ptrdiff_t> A_CCM;
  A_CCM.fromDense(A);
  cholmod_sparse A_CS;
  A_CCM.getView(&A_CS);
  cholmod_dense b_CD;
  aslam::calibration::eigenDenseToCholmodDenseView(b, &b_CD);
  Eigen::VectorXd x_est;
  linearSolver.solve(&A_CS, &b_CD, 29, x_est);
  ASSERT_NEAR((b - A * x_est).norm(), 0, 1e-9);
}
