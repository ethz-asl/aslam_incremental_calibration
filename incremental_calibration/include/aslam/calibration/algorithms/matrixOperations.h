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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file matrixOperations.h
    \brief This file defines some useful matrix operations.
  */

#ifndef ASLAM_CALIBRATION_MATRIX_OPERATIONS_H
#define ASLAM_CALIBRATION_MATRIX_OPERATIONS_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {
    /// Computes a covariance block from the R matrix of QR decomposition
    template <typename T>
    Eigen::MatrixXd computeCovariance(const T& R, size_t colBegin,
      size_t colEnd);

    /// Computes the sum of the log of the diagonal elements of R from QR
    template <typename T>
    double computeSumLogDiagR(const T& R, size_t colBegin, size_t colEnd);

    /// Checks column indices
    template <typename T>
    void checkColumnIndices(const T& R, size_t colBegin, size_t colEnd);

  }
}

#include "aslam/calibration/algorithms/matrixOperations.tpp"

#endif // ASLAM_CALIBRATION_MATRIX_OPERATIONS_H
