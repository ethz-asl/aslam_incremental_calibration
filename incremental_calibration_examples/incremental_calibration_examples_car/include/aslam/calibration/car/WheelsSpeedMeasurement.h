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

/** \file WheelsSpeedMeasurement.h
    \brief This file defines the WheelsSpeedMeasurement structure which
           represents a wheels speed measurement.
  */

#ifndef ASLAM_CALIBRATION_WHEELS_SPEED_MEASUREMENT_H
#define ASLAM_CALIBRATION_WHEELS_SPEED_MEASUREMENT_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    /** The structure WheelsSpeedMeasurement represents a wheels speed
        measurement.
        \brief Wheels speed measurement.
      */
    struct WheelsSpeedMeasurement {
      /** \name Public members
        @{
        */
      /// Left speed measurement
      uint16_t left;
      /// Right speed measurement
      uint16_t right;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_WHEELS_SPEED_MEASUREMENT_H
