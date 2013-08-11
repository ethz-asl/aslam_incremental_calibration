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

#include "aslam/calibration/car/MeasurementsContainer.h"

namespace sm {
  namespace kinematics {

    class Transformation;

  }
}

namespace aslam {

  class DiscreteTrajectory;
  class SplineTrajectory;

  namespace calibration {

    struct ApplanixNavigationMeasurement;
    struct ApplanixDMIMeasurement;
    struct WheelsSpeedMeasurement;
    struct SteeringMeasurement;

    /** \name Methods
      @{
      */
    /// Ensures rotation vector does not flip
    Eigen::Vector3d bestRotVector(const Eigen::Vector3d& prv, const
      Eigen::Vector3d& crv);
    /// Finds the best quaternion
    Eigen::Vector4d bestQuat(const Eigen::Vector4d& pquat, const
      Eigen::Vector4d& cquat);
    /// Builds a discrete trajectory based on Applanix navigation measurements
    void generateTrajectory(const MeasurementsContainer<
      ApplanixNavigationMeasurement>::Type& measurements, DiscreteTrajectory&
      trajectory);
    /// Simulates rear wheels speed measurements
    void simulateRearWheelsSpeedMeasurements(const SplineTrajectory& trajectory,
      double frequency, double sigma2_rl, double sigma2_rr, double e_r, double
      k_rl, double k_rr, const sm::kinematics::Transformation& T_io,
      MeasurementsContainer<WheelsSpeedMeasurement>::Type& trueMeasurements,
      MeasurementsContainer<WheelsSpeedMeasurement>::Type& noisyMeasurements);
    /// Simulates front wheels speed measurements
    void simulateFrontWheelsSpeedMeasurements(const SplineTrajectory&
      trajectory, double frequency, double sigma2_fl, double sigma2_fr, double
      e_f, double L, double k_fl, double k_fr, const
      sm::kinematics::Transformation& T_io,
      MeasurementsContainer<WheelsSpeedMeasurement>::Type& trueMeasurements,
      MeasurementsContainer<WheelsSpeedMeasurement>::Type& noisyMeasurements);
    /// Simulates steering measurements
    void simulateSteeringMeasurements(const SplineTrajectory& trajectory, double
      frequency, double sigma2_st, double L, double a0, double a1, double a2,
      double a3, const sm::kinematics::Transformation& T_io,
      MeasurementsContainer<SteeringMeasurement>::Type& trueMeasurements,
      MeasurementsContainer<SteeringMeasurement>::Type& noisyMeasurements);
    /// Simulates DMI measurements
    void simulateDMIMeasurements(const SplineTrajectory& trajectory, double
      frequency, double sigma2_dmi, double e_r, const
      sm::kinematics::Transformation& T_io,
      MeasurementsContainer<ApplanixDMIMeasurement>::Type& trueMeasurements,
      MeasurementsContainer<ApplanixDMIMeasurement>::Type& noisyMeasurements);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_CAR_UTILS_H
