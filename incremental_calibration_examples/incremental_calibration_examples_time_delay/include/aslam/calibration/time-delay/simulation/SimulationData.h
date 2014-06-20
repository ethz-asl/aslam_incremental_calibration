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

/** \file SimulationData.h
    \brief This file represents simulation data.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_DATA_H
#define ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_DATA_H

#include <fstream>
#include <vector>

#include "aslam/calibration/time-delay/simulation/Trajectory.h"
#include "aslam/calibration/time-delay/data/MeasurementsContainer.h"
#include "aslam/calibration/time-delay/data/WheelSpeedMeasurement.h"
#include "aslam/calibration/time-delay/data/PoseMeasurement.h"

namespace aslam {
  namespace calibration {

    /** The structure SimulationData holds the simulation data.
        \brief Simulation data
      */
    struct SimulationData {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Members
        @{
        */
      /// Vehicle's trajectory
      Trajectory trajectory;
      /// Vector of left wheel linear velocities
      std::vector<Eigen::Vector3d> w_v_wwl;
      /// Vector of right wheel linear velocities
      std::vector<Eigen::Vector3d> w_v_wwr;
      /// Left wheel noise-free measurement
      MeasurementsContainer<WheelSpeedMeasurement>::Type lwData;
      /// Left wheel noisy measurement
      MeasurementsContainer<WheelSpeedMeasurement>::Type lwData_n;
      /// Right wheel noise-free measurement
      MeasurementsContainer<WheelSpeedMeasurement>::Type rwData;
      /// Right wheel noisy measurement
      MeasurementsContainer<WheelSpeedMeasurement>::Type rwData_n;
      /// Pose noise-free measurement
      MeasurementsContainer<PoseMeasurement>::Type poseData;
      /// Pose noisy measurement
      MeasurementsContainer<PoseMeasurement>::Type poseData_n;
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Output w_v_wwl
      void w_v_wwlWrite(std::ofstream& stream) const;
      /// Output w_v_wwr
      void w_v_wwrWrite(std::ofstream& stream) const;

      /// Output lwData
      void lwDataWrite(std::ofstream& stream) const;
      /// Output lwData_n
      void lwData_nWrite(std::ofstream& stream) const;
      /// Output rwData
      void rwDataWrite(std::ofstream& stream) const;
      /// Output rwData_n
      void rwData_nWrite(std::ofstream& stream) const;
      /// Output poseData
      void poseDataWrite(std::ofstream& stream) const;
      /// Output poseData_n
      void poseData_nWrite(std::ofstream& stream) const;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_DATA_H
