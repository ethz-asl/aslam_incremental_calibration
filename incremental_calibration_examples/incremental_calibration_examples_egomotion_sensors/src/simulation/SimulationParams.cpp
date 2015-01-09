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

#include "aslam/calibration/egomotion/simulation/SimulationParams.h"

#include <sstream>
#include <string>

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

    SimulationParams::SimulationParams(const PropertyTree& config, const
        PropertyTree& parent) {
      referenceSensor = parent.getInt("egomotion/calibrator/referenceSensor");

      auto numSensors = config.getInt("calibration/num");
      sensorCalibration.reserve(numSensors);
      for (int i = 0; i < numSensors; ++i) {
        std::stringstream tagStream;
        tagStream << "calibration/sensor_" << i << "/";
        const auto tag = tagStream.str();
        const EulerAnglesYawPitchRoll ypr;
        auto T = rt2Transform(ypr.parametersToRotationMatrix(
          Eigen::Vector3d(config.getDouble(tag + "extrinsics/rotation/yaw"),
          config.getDouble(tag + "extrinsics/rotation/pitch"),
          config.getDouble(tag + "extrinsics/rotation/roll"))),
          Eigen::Vector3d(
          config.getDouble(tag + "extrinsics/translation/x"),
          config.getDouble(tag + "extrinsics/translation/y"),
          config.getDouble(tag + "extrinsics/translation/z")));
        auto t = config.getInt(tag + "intrinsics/timeDelay");
        auto idx = config.getInt(tag + "idx");
        sensorCalibration[idx] = std::make_pair(t, T);
      }

      dt = config.getDouble("dt");
      T = config.getDouble("T");

      sigma2.reserve(numSensors + 1);
      for (int i = 0; i < numSensors + 1; ++i) {
        std::stringstream tagStream;
        tagStream << "noise/sensor_" << i << "/";
        const auto tag = tagStream.str();
        auto idx = config.getInt(tag + "idx");
        Eigen::Matrix<double, 6, 6> cov =
          Eigen::Matrix<double, 6, 6>::Zero();
        cov(0, 0) = config.getDouble(tag + "e1");
        cov(1, 1) = config.getDouble(tag + "e2");
        cov(2, 2) = config.getDouble(tag + "e3");
        cov(3, 3) = config.getDouble(tag + "e4");
        cov(4, 4) = config.getDouble(tag + "e5");
        cov(5, 5) = config.getDouble(tag + "e6");
        sigma2[idx] = cov;
      }

      trajectoryParams = TrajectoryParams(sm::PropertyTree(config,
        "trajectory"));

    }

  }
}
