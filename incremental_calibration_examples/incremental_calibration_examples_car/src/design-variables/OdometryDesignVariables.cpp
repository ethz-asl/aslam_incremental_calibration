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

#include "aslam/calibration/car/design-variables/OdometryDesignVariables.h"

#include <boost/make_shared.hpp>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
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
      e_r = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/halfRearTrack"));
      e_r->setActive(true);
      e_f = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/halfFrontTrack"));
      e_f->setActive(true);
      L = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/wheelBase"));
      L->setActive(true);

      a = boost::make_shared<VectorDesignVariable<4> >(
        (VectorDesignVariable<4>::Container() <<
        config.getDouble("intrinsics/steeringCoefficient0"),
        config.getDouble("intrinsics/steeringCoefficient1"),
        config.getDouble("intrinsics/steeringCoefficient2"),
        config.getDouble("intrinsics/steeringCoefficient3")
        ).finished());
      a->setActive(true);

      k_rl = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/rlwCoefficient"));
      k_rl->setActive(true);
      k_rr = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/rrwCoefficient"));
      k_rr->setActive(true);
      k_fl = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/flwCoefficient"));
      k_fl->setActive(true);
      k_fr = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/frwCoefficient"));
      k_fr->setActive(true);
      k_dmi = boost::make_shared<Scalar>(config.getDouble(
        "intrinsics/dmiCoefficient"));
      k_dmi->setActive(true);

      v_r_vr = boost::make_shared<EuclideanPoint>(Eigen::Vector3d(
        config.getDouble("extrinsics/translation/x"),
        config.getDouble("extrinsics/translation/y"),
        config.getDouble("extrinsics/translation/z")));
      v_r_vr->setActive(false);
      const EulerAnglesYawPitchRoll ypr;
      v_R_r = boost::make_shared<RotationQuaternion>(
        ypr.parametersToRotationMatrix(
        Eigen::Vector3d(config.getDouble("extrinsics/rotation/yaw"),
        config.getDouble("extrinsics/rotation/pitch"),
        config.getDouble("extrinsics/rotation/roll"))));
      v_R_r->setActive(false);

      t_r = boost::make_shared<TimeDesignVariable>(
        config.getInt("timeDelays/rearWheels"));
      t_f = boost::make_shared<TimeDesignVariable>(
        config.getInt("timeDelays/frontWheels"));
      t_s = boost::make_shared<TimeDesignVariable>(
        config.getInt("timeDelays/steering"));
      t_dmi = boost::make_shared<TimeDesignVariable>(
        config.getInt("timeDelays/dmi"));
      if (config.getBool("timeDelays/active")) {
        t_r->setActive(true);
        t_f->setActive(true);
        t_s->setActive(true);
        t_dmi->setActive(true);
      }
      else {
        t_r->setActive(false);
        t_f->setActive(false);
        t_s->setActive(false);
        t_dmi->setActive(false);
      }
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OdometryDesignVariables::addToBatch(const BatchSP& batch, size_t
        groupId) {
      batch->addDesignVariable(e_r, groupId);
      batch->addDesignVariable(e_f, groupId);
      batch->addDesignVariable(L, groupId);
      batch->addDesignVariable(a, groupId);
      batch->addDesignVariable(k_rl, groupId);
      batch->addDesignVariable(k_rr, groupId);
      batch->addDesignVariable(k_fl, groupId);
      batch->addDesignVariable(k_fr, groupId);
      batch->addDesignVariable(k_dmi, groupId);
      batch->addDesignVariable(v_r_vr, groupId);
      batch->addDesignVariable(v_R_r, groupId);
      batch->addDesignVariable(t_r, groupId);
      batch->addDesignVariable(t_f, groupId);
      batch->addDesignVariable(t_s, groupId);
      batch->addDesignVariable(t_dmi, groupId);
    }

    Eigen::VectorXd OdometryDesignVariables::getParameters() const {
      Eigen::VectorXd params(22);
      params(0) = e_r->getParameters()(0);
      params(1) = e_f->getParameters()(0);
      params(2) = L->getParameters()(0);
      params(3) = k_rl->getParameters()(0);
      params(4) = k_rr->getParameters()(0);
      params(5) = k_fl->getParameters()(0);
      params(6) = k_fr->getParameters()(0);
      params(7) = k_dmi->getParameters()(0);
      params.segment<4>(8) = a->getValue();
      params.segment<3>(12) = v_r_vr->toEuclidean();
      const EulerAnglesYawPitchRoll ypr;
      params.segment<3>(15) = ypr.rotationMatrixToParameters(
        quat2r(v_R_r->getQuaternion()));
      params(18) = t_r->getParameters()(0);
      params(19) = t_f->getParameters()(0);
      params(20) = t_s->getParameters()(0);
      params(11) = t_dmi->getParameters()(0);
      return params;
    }

/******************************************************************************/
/* Stream methods                                                             */
/******************************************************************************/

    void OdometryDesignVariables::read(std::istream& stream) {
    }

    void OdometryDesignVariables::write(std::ostream& stream) const {
      stream << "e_r: " << e_r->getParameters() << std::endl;
      stream << "e_f: " << e_f->getParameters() << std::endl;
      stream << "L: " << L->getParameters() << std::endl;
      stream << "a: " << *a << std::endl;
      stream << "k_rl: " << k_rl->getParameters() << std::endl;
      stream << "k_rr: " << k_rr->getParameters() << std::endl;
      stream << "k_fl: " << k_fl->getParameters() << std::endl;
      stream << "k_fr: " << k_fr->getParameters() << std::endl;
      stream << "k_dmi: " << k_dmi->getParameters() << std::endl;
      stream << "v_r_vr: " << v_r_vr->toEuclidean().transpose() << std::endl;
      const EulerAnglesYawPitchRoll ypr;
      stream << "v_R_r: " << ypr.rotationMatrixToParameters(
        quat2r(v_R_r->getQuaternion())).transpose() << std::endl;
      stream << "t_r: " << t_r->getParameters() << std::endl;
      stream << "t_f: " << t_f->getParameters() << std::endl;
      stream << "t_s: " << t_s->getParameters() << std::endl;
      stream << "t_dmi: " << t_dmi->getParameters() << std::endl;
    }

    void OdometryDesignVariables::read(std::ifstream& stream) {
    }

    void OdometryDesignVariables::write(std::ofstream& stream) const {
    }

  }
}
