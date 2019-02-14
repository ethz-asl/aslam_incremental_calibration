/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file bestQuat.h
    \brief This file contains a utility to avoid jumps in quaternions.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_BESTQUAT_H
#define ASLAM_CALIBRATION_TIME_DELAY_BESTQUAT_H

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
    /// Finds the best quaternion avoiding jumps
    Eigen::Vector4d bestQuat(const Eigen::Vector4d& pquat, const
      Eigen::Vector4d& cquat);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_BESTQUAT_H
