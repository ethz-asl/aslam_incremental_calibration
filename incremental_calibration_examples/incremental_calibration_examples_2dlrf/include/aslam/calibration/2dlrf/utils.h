/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file utils.h
    \brief This file contains a bunch of utilities for the 2D-LRF problem.
  */

#ifndef ASLAM_CALIBRATION_2DLRF_UTILS_H
#define ASLAM_CALIBRATION_2DLRF_UTILS_H

#include <vector>

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /// Generate a sine wave path
    void genSineWavePath(std::vector<Eigen::Matrix<double, 3, 1> >& u,
      size_t steps, double amplitude, double frequency, double T);
    /// Inits landmarks positions from noisy data
    void initLandmarks(std::vector<Eigen::Matrix<double, 2, 1> >& x_l_hat,
      const std::vector<Eigen::Matrix<double, 3, 1> >& x_odom,
      const Eigen::Matrix<double, 3, 1>& Theta_hat,
      const std::vector<std::vector<double> >& r,
      const std::vector<std::vector<double> >& b);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_2DLRF_UTILS_H
