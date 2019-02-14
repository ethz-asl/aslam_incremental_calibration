/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
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
