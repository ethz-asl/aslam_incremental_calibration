/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
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

#include "aslam/calibration/time-delay/algo/CalibratorOptions.h"

#include <sm/PropertyTree.hpp>

using namespace sm;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CalibratorOptions::CalibratorOptions() :
        windowDuration(10.0),
        transSplineLambda(0.0),
        rotSplineLambda(0.0),
        splineKnotsPerSecond(5),
        transSplineOrder(4),
        rotSplineOrder(4),
        lwVariance(1e-3),
        rwVariance(1e-3),
        vyVariance(1e-1),
        vzVariance(1e-1),
        verbose(true),
        useTimeDelay(true) {
    }

    CalibratorOptions::CalibratorOptions(const PropertyTree& config) {
      windowDuration = config.getDouble("windowDuration");
      verbose = config.getBool("verbose");
      useTimeDelay = config.getBool("useTimeDelay");

      transSplineLambda = config.getDouble("splines/transSplineLambda");
      rotSplineLambda = config.getDouble("splines/rotSplineLambda");
      splineKnotsPerSecond = config.getInt("splines/splineKnotsPerSecond");
      transSplineOrder = config.getInt("splines/transSplineOrder",
        transSplineOrder);
      rotSplineOrder = config.getInt("splines/rotSplineOrder");

      lwVariance = config.getDouble("odometry/sensors/wss/noise/lwVariance");
      rwVariance = config.getDouble("odometry/sensors/wss/noise/rwVariance");
      vyVariance = config.getDouble(
        "odometry/constraints/noise/vyVariance");
      vzVariance = config.getDouble(
        "odometry/constraints/noise/vzVariance");
    }

  }
}
