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

/** \file ApplanixNavigationMeasurement.h
    \brief This file defines the ApplanixNavigationMeasurement structure which
           represents an Applanix navigation measurement.
  */

#ifndef ASLAM_CALIBRATION_APPLANIX_NAVIGATION_MEASUREMENT_H
#define ASLAM_CALIBRATION_APPLANIX_NAVIGATION_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure ApplanixNavigationMeasurement represents an Applanix
        navigation measurement in local ENU system.
        \brief Applanix navigation measurement.
      */
    struct ApplanixNavigationMeasurement {
      /** \name Public members
        @{
        */
      /// x pose [m]
      double x;
      /// y pose [m]
      double y;
      /// z pose [m]
      double z;
      /// roll [rad]
      double roll;
      /// pitch [rad]
      double pitch;
      /// yaw [rad]
      double yaw;
      /// body linear velocity in x in world frame [m/s]
      double v_x;
      /// body linear velocity in y in world frame [m/s]
      double v_y;
      /// body linear velocity in z in world frame [m/s]
      double v_z;
      /// body angular velocity in x in body frame [rad/s]
      double om_x;
      /// body angular velocity in y in body frame [rad/s]
      double om_y;
      /// body angular velocity in z in body frame [rad/s]
      double om_z;
      /// body linear acceleration in x in body frame [m/s^2]
      double a_x;
      /// body linear acceleration in y in body frame [m/s^2]
      double a_y;
      /// body linear acceleration in z in body frame [m/s^2]
      double a_z;
      /// linear velocity [m/s]
      double v;
      /// x pose sigma^2
      double x_sigma2;
      /// y pose sigma^2
      double y_sigma2;
      /// z pose sigma^2
      double z_sigma2;
      /// roll sigma^2
      double roll_sigma2;
      /// pitch sigma^2
      double pitch_sigma2;
      /// yaw sigma^2
      double yaw_sigma2;
      /// linear velocity in x sigma^2
      double v_x_sigma2;
      /// linear velocity in y sigma^2
      double v_y_sigma2;
      /// linear velocity in z sigma^2
      double v_z_sigma2;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_APPLANIX_NAVIGATION_MEASUREMENT_H
