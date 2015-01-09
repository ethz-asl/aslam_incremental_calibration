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

#include "aslam/calibration/egomotion/simulation/TrajectoryParams.h"

#include <sm/PropertyTree.hpp>

#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace sm;
using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    TrajectoryParams::TrajectoryParams() :
        w_T_v_0(),
        A(15.0),
        f(0.01),
        transSplineLambda(0.0),
        rotSplineLambda(0.0),
        splineKnotsPerSecond(5),
        transSplineOrder(4),
        rotSplineOrder(4) {
    }

    TrajectoryParams::TrajectoryParams(const PropertyTree& config) {
      A = config.getDouble("A");
      f = config.getDouble("f");
      const EulerAnglesYawPitchRoll ypr;
      w_T_v_0 = rt2Transform(ypr.parametersToRotationMatrix(Eigen::Vector3d(
        config.getDouble("w_T_v_0/yaw"), config.getDouble("w_T_v_0/pitch"),
        config.getDouble("w_T_v_0/roll"))), Eigen::Vector3d(
        config.getDouble("w_T_v_0/x"), config.getDouble("w_T_v_0/y"),
        config.getDouble("w_T_v_0/z")));
      transSplineLambda = config.getDouble("splines/transSplineLambda");
      rotSplineLambda = config.getDouble("splines/rotSplineLambda");
      splineKnotsPerSecond = config.getInt("splines/splineKnotsPerSecond");
      transSplineOrder = config.getInt("splines/transSplineOrder",
        transSplineOrder);
      rotSplineOrder = config.getInt("splines/rotSplineOrder");
      type = config.getString("type");
    }

  }
}
