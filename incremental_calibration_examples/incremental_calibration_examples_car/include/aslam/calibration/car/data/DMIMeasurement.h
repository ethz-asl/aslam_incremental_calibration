/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

/** \file DMIMeasurement.h
    \brief This file defines the DMIMeasurement structure which represents an
           Applanix DMI measurement.
  */

#ifndef ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H
#define ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure DMIMeasurement represents an Applanix DMI
        measurement.
        \brief Applanix DMI measurement.
      */
    struct DMIMeasurement {
      /** \name Public members
        @{
        */
      /// Wheel speed
      double wheelSpeed;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H
