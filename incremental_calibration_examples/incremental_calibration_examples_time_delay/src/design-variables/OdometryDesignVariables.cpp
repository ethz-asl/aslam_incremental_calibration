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

#include "aslam/calibration/time-delay/design-variables/OdometryDesignVariables.h"

#include <boost/make_shared.hpp>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>

using namespace sm;
using namespace sm::kinematics;
using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    OdometryDesignVariables::OdometryDesignVariables(const PropertyTree&
        config) {
      b = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/halfWheelBase"));
      b->setActive(true);

      k_l = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/lwCoefficient"));
      k_l->setActive(true);
      k_r = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/rwCoefficient"));
      k_r->setActive(true);

      v_r_vp = boost::make_shared<EuclideanPoint>(Eigen::Vector3d(
        config.getDouble("extrinsics/translation/x"),
        config.getDouble("extrinsics/translation/y"),
        config.getDouble("extrinsics/translation/z")));
      if (config.getBool("extrinsics/translation/active"))
        v_r_vp->setActive(true);
      else
        v_r_vp->setActive(false);
      const EulerAnglesYawPitchRoll ypr;
      v_R_p = boost::make_shared<RotationQuaternion>(
        ypr.parametersToRotationMatrix(
        Eigen::Vector3d(config.getDouble("extrinsics/rotation/yaw"),
        config.getDouble("extrinsics/rotation/pitch"),
        config.getDouble("extrinsics/rotation/roll"))));
      if (config.getBool("extrinsics/rotation/active"))
        v_R_p->setActive(true);
      else
        v_R_p->setActive(false);

      t_l = boost::make_shared<TimeDesignVariable>(
        config.getInt("intrinsics/lwTimeDelay"));
      t_r = boost::make_shared<TimeDesignVariable>(
        config.getInt("intrinsics/rwTimeDelay"));
      if (config.getBool("intrinsics/timeDelaysActive")) {
        t_l->setActive(true);
        t_r->setActive(true);
      }
      else {
        t_l->setActive(false);
        t_r->setActive(false);
      }

    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OdometryDesignVariables::addToBatch(const BatchSP& batch, size_t
        groupId) {
      batch->addDesignVariable(b, groupId);
      batch->addDesignVariable(k_l, groupId);
      batch->addDesignVariable(t_l, groupId);
      batch->addDesignVariable(k_r, groupId);
      batch->addDesignVariable(t_r, groupId);
      batch->addDesignVariable(v_r_vp, groupId);
      batch->addDesignVariable(v_R_p, groupId);
    }

    Eigen::VectorXd OdometryDesignVariables::getParameters() const {
      Eigen::VectorXd params(11);
      params(0) = b->getParameters()(0);
      params(1) = k_l->getParameters()(0);
      params(2) = t_l->getParameters()(0);
      params(3) = k_r->getParameters()(0);
      params(4) = t_r->getParameters()(0);
      params.segment<3>(5) = v_r_vp->toEuclidean();
      const EulerAnglesYawPitchRoll ypr;
      params.segment<3>(8) = ypr.rotationMatrixToParameters(
        quat2r(v_R_p->getQuaternion()));
      return params;
    }

/******************************************************************************/
/* Stream methods                                                             */
/******************************************************************************/

    void OdometryDesignVariables::read(std::istream& stream) {
    }

    void OdometryDesignVariables::write(std::ostream& stream) const {
      stream << "b: " << b->getParameters() << std::endl;
      stream << "k_l: " << k_l->getParameters() << std::endl;
      stream << "t_l: " << t_l->getParameters() << std::endl;
      stream << "k_r: " << k_r->getParameters() << std::endl;
      stream << "t_r: " << t_r->getParameters() << std::endl;
      stream << "v_r_vp: " << v_r_vp->toEuclidean().transpose() << std::endl;
      const EulerAnglesYawPitchRoll ypr;
      stream << "v_R_p: " << ypr.rotationMatrixToParameters(
        quat2r(v_R_p->getQuaternion())).transpose() << std::endl;
    }

    void OdometryDesignVariables::read(std::ifstream& stream) {
    }

    void OdometryDesignVariables::write(std::ofstream& stream) const {
    }

  }
}
