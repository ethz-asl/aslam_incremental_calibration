/******************************************************************************
 * Copyright (C) 2012 by Jerome Maye                                          *
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

/** \file utils.h
    \brief This file contains a bunch of utilities for the 2D-LRF problem.
  */

#ifndef ASLAM_CALIBRATION_2DLRF_UTILS_H
#define ASLAM_CALIBRATION_2DLRF_UTILS_H

#include <vector>

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /// Generate a sine wave path
    void genSineWavePath(std::vector<Eigen::Matrix<double, 3, 1> >& u,
      size_t steps, double amplitude, double frequency, double T);

    /// Inits landmarks positions from noisy data
    void initLandmarks(std::vector<Eigen::Matrix<double, 2, 1> >& x_l_hat,
      const std::vector<Eigen::Matrix<double, 3, 1> >& x_odom,
      const Eigen::Matrix<double, 3, 1>& Theta_hat,
      const std::vector<std::vector<double> >& r,
      const std::vector<std::vector<double> >& b);

  }
}

#endif // ASLAM_CALIBRATION_2DLRF_UTILS_H
