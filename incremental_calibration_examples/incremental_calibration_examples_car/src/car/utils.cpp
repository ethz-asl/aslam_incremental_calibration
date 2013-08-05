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

#include "aslam/calibration/car/utils.h"

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    Eigen::Vector3d bestRotVector(const Eigen::Vector3d& prv, const
        Eigen::Vector3d& crv) {
      // current angle
      const double angle = crv.norm();
      // current axis
      const Eigen::Vector3d axis = crv / angle;
      // best rotation vector
      Eigen::Vector3d best_rv = crv;
      // best distance
      double best_dist = (best_rv - prv).norm();
      // find best vector
      for (int s = -3; s <= 4; ++s) {
        const Eigen::Vector3d aa = axis * (angle + M_PI * 2.0 * s);
        const double dist = (aa - prv).norm();
        if (dist < best_dist) {
          best_rv = aa;
          best_dist = dist;
        }
      }
      return best_rv;
    }

    Eigen::Vector4d bestQuat(const Eigen::Vector4d& pquat, const
        Eigen::Vector4d& cquat) {
      if ((pquat + cquat).norm() < (pquat - cquat).norm())
        return -cquat;
      else
        return cquat;
    }

    void generateTrajectory(const MeasurementsContainer<
        ApplanixNavigationMeasurement>::Type& measurements, DiscreteTrajectory&
        trajectory) {
      const sm::kinematics::EulerAnglesYawPitchRoll ypr;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it)
        trajectory.addPose(it->first * 1e9, sm::kinematics::Transformation(
          sm::kinematics::r2quat(ypr.parametersToRotationMatrix(Eigen::Vector3d(
          it->second.yaw, it->second.pitch, it->second.roll))), Eigen::Vector3d(
          it->second.x, it->second.y, it->second.z)));
    }

    void simulateRearWheelsSpeedMeasurements(const SplineTrajectory& trajectory,
        double frequency, double sigma2_t, double sigma2_rl, double sigma2_rr,
        double e_r, double k_rl, double k_rr, const
        sm::kinematics::Transformation& T_io,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type& measurements) {
    }

    void simulateFrontWheelsSpeedMeasurements(const SplineTrajectory&
        trajectory, double frequency, double sigma2_t, double sigma2_fl, double
        sigma2_fr, double e_f, double L, double k_fl, double k_fr, const
        sm::kinematics::Transformation& T_io,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type& measurements) {
    }

    void simulateSteeringMeasurements(const SplineTrajectory& trajectory, double
        frequency, double sigma2_t, double sigma2_st, double L, double a0,
        double a1, double a2, double a3, const sm::kinematics::Transformation&
        T_io, MeasurementsContainer<SteeringMeasurement>::Type& measurements) {
    }

    void simulateDMIMeasurements(const SplineTrajectory& trajectory, double
        frequency, double sigma2_t, double sigma2_dmi, double e_r, const
        sm::kinematics::Transformation& T_io,
        MeasurementsContainer<ApplanixDMIMeasurement>::Type& measurements) {
    }

  }
}
