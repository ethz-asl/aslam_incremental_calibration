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

/** \file utils.h
    \brief This file contains a bunch of utilities for the car problem.
  */

#ifndef ASLAM_CALIBRATION_CAR_UTILS_H
#define ASLAM_CALIBRATION_CAR_UTILS_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /// Ensures rotation vector does not flip
    Eigen::Vector3d rotVectorNoFlipping(const Eigen::Vector3d& prv,
      const Eigen::Vector3d& crv);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_CAR_UTILS_H
