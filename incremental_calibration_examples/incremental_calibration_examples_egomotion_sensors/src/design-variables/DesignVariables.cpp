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

#include "aslam/calibration/egomotion/design-variables/DesignVariables.h"

#include <sstream>
#include <string>

#include <boost/make_shared.hpp>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/exceptions/OutOfBoundException.h>

using namespace sm;
using namespace sm::kinematics;
using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    DesignVariables::DesignVariables(const PropertyTree& config) {
      auto numSensors = config.getInt("num");
      calibrationVariables_.reserve(numSensors);
      for (int i = 0; i < numSensors; ++i) {
        std::stringstream tagStream;
        tagStream << "sensor_" << i << "/";
        std::string tag = tagStream.str();
        auto r = boost::make_shared<EuclideanPoint>(Eigen::Vector3d(
          config.getDouble(tag + "extrinsics/translation/x"),
          config.getDouble(tag + "extrinsics/translation/y"),
          config.getDouble(tag + "extrinsics/translation/z")));
        if (config.getBool(tag + "extrinsics/translation/active"))
          r->setActive(true);
        else
          r->setActive(false);
        const EulerAnglesYawPitchRoll ypr;
        auto R = boost::make_shared<RotationQuaternion>(
          ypr.parametersToRotationMatrix(
          Eigen::Vector3d(config.getDouble(tag + "extrinsics/rotation/yaw"),
          config.getDouble(tag + "extrinsics/rotation/pitch"),
          config.getDouble(tag + "extrinsics/rotation/roll"))));
        if (config.getBool(tag + "extrinsics/rotation/active"))
          R->setActive(true);
        else
          R->setActive(false);
        auto t = boost::make_shared<TimeDesignVariable>(
          config.getInt(tag + "intrinsics/timeDelay"));
        if (config.getBool(tag + "intrinsics/timeDelayActive"))
          t->setActive(true);
        else
          t->setActive(false);
        auto idx = config.getInt(tag + "idx");
        calibrationVariables_[idx] = std::make_tuple(t, r, R);
      }
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void DesignVariables::addToBatch(const BatchSP& batch, size_t groupId) {
      for (const auto& designVariable : calibrationVariables_) {
        batch->addDesignVariable(std::get<0>(designVariable.second), groupId);
        batch->addDesignVariable(std::get<1>(designVariable.second), groupId);
        batch->addDesignVariable(std::get<2>(designVariable.second), groupId);
      }
    }

    Eigen::VectorXd DesignVariables::getParameters(size_t idx) const {
      if (!calibrationVariables_.count(idx))
        throw OutOfBoundException<size_t>(idx,
          "DesignVariables::getParameters(): inexistent design variable");
      const auto& designVariable = calibrationVariables_.at(idx);
      Eigen::VectorXd params(7);
      params(0) = std::get<0>(designVariable)->getParameters()(0);
      params.segment<3>(1) = std::get<1>(designVariable)->toEuclidean();
      const EulerAnglesYawPitchRoll ypr;
      params.segment<3>(4) = ypr.rotationMatrixToParameters(
        quat2r(std::get<2>(designVariable)->getQuaternion()));
      return params;
    }

/******************************************************************************/
/* Stream methods                                                             */
/******************************************************************************/

    void DesignVariables::read(std::istream& /*stream*/) {
    }

    void DesignVariables::write(std::ostream& stream) const {
      for (const auto& designVariable : calibrationVariables_) {
        stream << "t_" << designVariable.first << ": "
          << std::get<0>(designVariable.second)->getParameters() << std::endl;
        stream << "r_" << designVariable.first << ": "
          << std::get<1>(designVariable.second)->toEuclidean().transpose()
          << std::endl;
        const EulerAnglesYawPitchRoll ypr;
        stream << "R_" << designVariable.first << ": "
          << ypr.rotationMatrixToParameters(quat2r(std::get<2>(
          designVariable.second)->getQuaternion())).transpose() << std::endl;
      }
    }

    void DesignVariables::read(std::ifstream& /*stream*/) {
    }

    void DesignVariables::write(std::ofstream& /*stream*/) const {
    }

  }
}
