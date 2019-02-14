/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file PoseMeasurement.h
    \brief This file defines the PoseMeasurement structure which represents a
           measurement of a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_POSE_MEASUREMENT_H
#define ASLAM_CALIBRATION_TIME_DELAY_POSE_MEASUREMENT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The structure PoseMeasurement represents a measurement returned by a
        pose sensor such as Applanix.
        \brief Pose measurement
      */
    struct PoseMeasurement {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Public members
        @{
        */
      /// Relative position of the reference frame w.r. to the world frame
      Eigen::Vector3d w_r_wp;
      /// Covariance matrix for w_r_wp
      Eigen::Matrix3d sigma2_w_r_wp;
      /// Relative orientation of the reference frame w.r. to the world frame
      Eigen::Vector3d w_R_p; // [yaw, pitch, roll]
      /// Covariance matrix for m_R_r
      Eigen::Matrix3d sigma2_w_R_p;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_POSE_MEASUREMENT_H
