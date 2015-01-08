/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
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

/** \file SimulationData.h
    \brief This file represents simulation data.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_SIMULATION_DATA_H
#define ASLAM_CALIBRATION_EGOMOTION_SIMULATION_DATA_H

#include <unordered_map>

#include "aslam/calibration/egomotion/simulation/Trajectory.h"
#include "aslam/calibration/egomotion/data/MeasurementsContainer.h"
#include "aslam/calibration/egomotion/data/MotionMeasurement.h"

namespace aslam {
  namespace calibration {

    /** The structure SimulationData holds the simulation data.
        \brief Simulation data
      */
    struct SimulationData {
      /** \name Members
        @{
        */
      /// Vehicle's trajectory
      Trajectory trajectory;
      /// Noise-free relative motion measurements
      std::unordered_map<size_t, MeasurementsContainer<MotionMeasurement>::Type>
        motionData;
      /// Noisy relative motion measurements
      std::unordered_map<size_t, MeasurementsContainer<MotionMeasurement>::Type>
        motionDataNoisy;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_SIMULATION_DATA_H
