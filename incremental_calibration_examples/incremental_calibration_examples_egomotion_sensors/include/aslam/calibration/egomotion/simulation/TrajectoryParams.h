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

/** \file TrajectoryParams.h
    \brief This file represents trajectory parameters.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_PARAMS_H
#define ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_PARAMS_H

#include <sm/kinematics/Transformation.hpp>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    /** The structure TrajectoryParams holds trajectory parameters
        \brief Trajectory
      */
    struct TrajectoryParams {
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      TrajectoryParams();
      /// Constructs options from property tree
      TrajectoryParams(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Members
        @{
        */
      /// Initial transformation
      sm::kinematics::Transformation w_T_v_0;
      /// Sine wave amplitude [m]
      double A;
      /// Sine wave frequency [Hz]
      double f;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_PARAMS_H
