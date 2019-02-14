/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file SimulationParams.h
    \brief This file contains simulation parameters.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_SIMULATION_PARAMS_H
#define ASLAM_CALIBRATION_EGOMOTION_SIMULATION_PARAMS_H

#include <unordered_map>
#include <utility>

#include <Eigen/Core>

#include <sm/kinematics/Transformation.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include "aslam/calibration/egomotion/simulation/TrajectoryParams.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    /** The structure SimulationParams holds simulation parameters.
        \brief Simulation parameters
      */
    struct SimulationParams {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Constructors/destructor
        @{
        */
      /// Constructs options from property tree
      SimulationParams(const sm::PropertyTree& config, const sm::PropertyTree&
        parent);
      /** @}
        */

      /** \name Members
        @{
        */
      /// Parameters for the trajectory
      TrajectoryParams trajectoryParams;
      /// Relative pose and time delay of the sensors w.r.t reference sensor
      std::unordered_map<size_t, std::pair<sm::timing::NsecTime,
        sm::kinematics::Transformation> > sensorCalibration;
      /// Simulation step size [s]
      double dt;
      /// Time of the simulation [s]
      double T;
      /// Covariance matrices of the measurements
      std::unordered_map<size_t, Eigen::Matrix<double, 6, 6> > sigma2;
      /// Reference sensor
      size_t referenceSensor;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_SIMULATION_PARAMS_H
