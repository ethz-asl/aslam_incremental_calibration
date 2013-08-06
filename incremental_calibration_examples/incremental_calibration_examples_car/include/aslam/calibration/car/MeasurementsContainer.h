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

/** \file MeasurementsContainer.h
    \brief This file defines a generic container for measurements.
  */

#ifndef ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H
#define ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H

#include <vector>
#include <utility>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace calibration {

    /** The structure MeasurementsContainer represents a generic measurements
        container.
        \brief Measurements container
      */
    template <typename C> struct MeasurementsContainer {
      /** \name Types definitions
        @{
        */
      /// Container type
      typedef std::vector<std::pair<sm::timing::NsecTime, C> > Type;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_MEASUREMENTS_CONTAINER_H
