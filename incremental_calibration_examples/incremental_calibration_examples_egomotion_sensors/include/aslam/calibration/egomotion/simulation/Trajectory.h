/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Trajectory.h
    \brief This file represents a trajectory.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_TRAJECTORY_H
#define ASLAM_CALIBRATION_EGOMOTION_TRAJECTORY_H

#include <boost/shared_ptr.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>

namespace aslam {
  namespace calibration {

    /** The structure Trajectory holds a trajectory.
        \brief Trajectory
      */
    struct Trajectory {
      /** \name Types definitions
        @{
        */
      /// Rotation spline
      typedef bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::TYPE RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Translation spline
      typedef bsplines::EuclideanBSpline<Eigen::Dynamic, 3,
        bsplines::NsecTimePolicy>::TYPE TranslationSpline;
      /// Euclidean spline shared pointer
      typedef boost::shared_ptr<TranslationSpline> TranslationSplineSP;
      /** @}
        */

      /** \name Members
        @{
        */
      /// Translation spline
      TranslationSplineSP translationSpline;
      /// Rotation spline
      RotationSplineSP rotationSpline;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_TRAJECTORY_H
