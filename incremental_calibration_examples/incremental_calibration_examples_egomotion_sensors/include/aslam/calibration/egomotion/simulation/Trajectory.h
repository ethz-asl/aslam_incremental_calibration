/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
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
