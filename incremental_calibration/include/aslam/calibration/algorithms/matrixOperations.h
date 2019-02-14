/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file matrixOperations.h
    \brief This file defines some useful matrix operations.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H
#define ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** 
     * This function computes the sum of the logarithm of the diagonal elements
     * of the input matrix.
     * \brief Sums the logarithm of the diagonal elements of the input matrix
     * 
     * \param[in] R input matrix
     * \param[in] colBegin column index where to start sum
     * \param[in] colEnd column index where to stop sum
     * \return sum of logarithm of diagonal elements
     */
    template <typename T>
    double computeSumLogDiagR(const T& R, size_t colBegin, size_t colEnd);

    /** 
     * This function checks the validity of input column index.
     * \brief Index checking
     * 
     * \param[in] R input matrix
     * \param[in] colBegin column index where to start sum
     * \param[in] colEnd column index where to stop sum
     */
    template <typename T>
    void checkColumnIndices(const T& R, size_t colBegin, size_t colEnd);
    /** @}
      */
  }
}

#include "aslam/calibration/algorithms/matrixOperations.tpp"
#endif // ASLAM_CALIBRATION_ALGORITHMS_MATRIX_OPERATIONS_H
