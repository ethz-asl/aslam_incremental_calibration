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

/** \file VelocitiesMeasurement.h
    \brief This file defines the VelocitiesMeasurement structure which
           represents a velocities measurement from a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_VELOCITIES_MEASUREMENT_H
#define ASLAM_CALIBRATION_VELOCITIES_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure VelocitiesMeasurement represents a velocities measurement
        returned by a pose sensor such as Applanix.
        \brief Velocities measurement
      */
    struct VelocitiesMeasurement {
      /** \name Public members
        @{
        */
      /// Linear velocity of sensor frame w.r. to mapping frame in sensor frame
      Eigen::Vector3d r_v_mr;
      /// Covariance matrix for r_v_mr
      Eigen::Matrix3d sigma2_r_v_mr;
      /// Angular velocity of sensor frame w.r. to mapping frame in sensor frame
      Eigen::Vector3d r_om_mr;
      /// Covariance matrix for r_om_mr
      Eigen::Matrix3d sigma2_r_om_mr;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_VELOCITIES_MEASUREMENT_H
