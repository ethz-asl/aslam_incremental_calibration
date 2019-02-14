/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file OuterProduct.h
    \brief This file defines the outer product functions
  */

#ifndef ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H
#define ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H

#include <cstdlib>

#include <Eigen/Core>

/** The OuterProduct namespace contains outer product functions.
    \brief Outer product functions
  */
namespace OuterProduct {
  /** \name Methods
    @{
    */
  /// The compute function generates the outer product of 2 vectors
  template <typename X, size_t M, size_t N>
  Eigen::Matrix<X, M, N> compute(const Eigen::Matrix<X, M, 1>& v1,
      const Eigen::Matrix<X, N, 1>& v2);
  /// The compute function generates the self outer product of a vector
  template <typename X, size_t M>
  Eigen::Matrix<X, M, M> compute(const Eigen::Matrix<X, M, 1>& v);
  /** @}
    */

}

#include "aslam/calibration/utils/OuterProduct.tpp"

#endif // ASLAM_CALIBRATION_UTILS_OUTERPRODUCT_H
