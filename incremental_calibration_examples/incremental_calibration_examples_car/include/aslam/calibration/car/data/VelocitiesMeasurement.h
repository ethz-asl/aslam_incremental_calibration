/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file VelocitiesMeasurement.h
    \brief This file defines the VelocitiesMeasurement structure which
           represents a velocities measurement from a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_CAR_VELOCITIES_MEASUREMENT_H
#define ASLAM_CALIBRATION_CAR_VELOCITIES_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure VelocitiesMeasurement represents a velocities measurement
        returned by a pose sensor such as Applanix.
        \brief Velocities measurement
      */
    struct VelocitiesMeasurement {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Public members
        @{
        */
      /// Linear velocity of sensor frame w.r. to mapping frame in sensor frame
      Eigen::Vector3d r_v_mr;
      /// Covariance matrix for r_v_mr
      Eigen::Matrix3d sigma2_r_v_mr;
      /// Angular velocity of sensor frame w.r. to mapping frame in sensor frame
      Eigen::Vector3d r_om_mr;
      /// Covariance matrix for r_om_mr
      Eigen::Matrix3d sigma2_r_om_mr;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_VELOCITIES_MEASUREMENT_H
