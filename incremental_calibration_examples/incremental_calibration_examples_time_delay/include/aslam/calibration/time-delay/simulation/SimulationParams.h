/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file SimulationParams.h
    \brief This file contains simulation parameters.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_PARAMS_H
#define ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_PARAMS_H

#include <Eigen/Core>

#include <sm/kinematics/Transformation.hpp>

#include "aslam/calibration/time-delay/simulation/TrajectoryParams.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    /** The structure SimulationParams holds simulation parameters.
        \brief Simulation parameters
      */
    struct SimulationParams {
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      SimulationParams();
      /// Constructs options from property tree
      SimulationParams(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Members
        @{
        */
      /// Parameters for the trajectory
      TrajectoryParams trajectoryParams;
      /// Relative pose of pose sensor w.r.t. to vehicle frame
      sm::kinematics::Transformation v_T_p;
      /// Wheel base
      double b;
      /// Left wheel scale factor
      double k_l;
      /// Time delay of left wheel sensor
      double t_l;
      /// Time delay of right wheel sensor
      double t_r;
      /// Right wheel scale factor
      double k_r;
      /// Simulation step size [s]
      double dt;
      /// Time of the simulation [s]
      double T;
      /// Variance of left wheel measurement
      double sigma2_l;
      /// Variance of right wheel measurement
      double sigma2_r;
      /// Covariance matrix for w_r_wp
      Eigen::Matrix3d sigma2_w_r_wp;
      /// Covariance matrix for w_R_p
      Eigen::Matrix3d sigma2_w_R_p;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_PARAMS_H
