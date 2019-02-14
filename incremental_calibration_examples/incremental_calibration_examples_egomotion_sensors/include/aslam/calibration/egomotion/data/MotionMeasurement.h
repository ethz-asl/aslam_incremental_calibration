/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file MotionMeasurement.h
    \brief This file defines the MotionMeasurement structure which represents
           the measurement of an incremental motion sensor.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_MOTION_MEASUREMENT_H
#define ASLAM_CALIBRATION_EGOMOTION_MOTION_MEASUREMENT_H

#include <Eigen/Core>

#include <sm/kinematics/Transformation.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace calibration {

    /** The structure MotionMeasurement represents a measurement returned by an
        incremental motion sensor.
        \brief Motion measurement
      */
    struct MotionMeasurement {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Public members
        @{
        */
      /// Incremental motion
      sm::kinematics::Transformation motion;
      /// Duration of the incremental motion
      sm::timing::NsecTime duration;
      /// Measurement covariance
      Eigen::Matrix<double, 6, 6> sigma2;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_MOTION_MEASUREMENT_H
