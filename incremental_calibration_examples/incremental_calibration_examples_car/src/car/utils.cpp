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

#include <cmath>
#include <limits>

#include <aslam/DiscreteTrajectory.hpp>
#include <aslam/SplineTrajectory.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/transformations.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/SteeringMeasurement.h"
#include "aslam/calibration/car/ApplanixDMIMeasurement.h"

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
        trajectory.addPose(it->first, sm::kinematics::Transformation(
          sm::kinematics::r2quat(ypr.parametersToRotationMatrix(Eigen::Vector3d(
          it->second.yaw, it->second.pitch, it->second.roll))), Eigen::Vector3d(
          it->second.x, it->second.y, it->second.z)));
    }

    void simulateRearWheelsSpeedMeasurements(const SplineTrajectory& trajectory,
        double frequency, double sigma2_rl, double sigma2_rr, double e_r, double
        k_rl, double k_rr, const sm::kinematics::Transformation& T_io,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type& trueMeasurements,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type&
        noisyMeasurements) {
      const sm::timing::NsecTime trajT_min = trajectory.minTime();
      const sm::timing::NsecTime trajT_max = trajectory.maxTime();
      const NormalDistribution<1> timeDist(0, 3e4);
      const sm::timing::NsecTime odoT_min = trajT_min +
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime odoT_max = trajT_max -
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime T = round(1.0 / frequency * 1e9);
      const NormalDistribution<1> rlDist(0, sigma2_rl);
      const NormalDistribution<1> rrDist(0, sigma2_rr);
      sm::timing::NsecTime t = odoT_min;
      while (t < odoT_max) {
        const Eigen::Matrix3d C_wi = trajectory.C(t);
        const Eigen::Vector3d v_ii = C_wi.transpose() *
          trajectory.linearVelocity(t);
        const Eigen::Vector3d om_ii = C_wi.transpose() *
          trajectory.angularVelocity(t);
        const Eigen::Vector3d v_oo = T_io.C().transpose() *
          (v_ii + om_ii.cross(T_io.t()));
        const Eigen::Vector3d om_oo = T_io.C().transpose() * om_ii;
        const double v_oo_x = v_oo(0);
        const double om_oo_z = om_oo(2);
        WheelsSpeedMeasurement trueData;
        trueData.left = fabs(round((v_oo_x - e_r * om_oo_z) / k_rl));
        trueData.right = fabs(round((v_oo_x + e_r * om_oo_z) / k_rr));
        trueMeasurements.push_back(std::make_pair(t, trueData));
        WheelsSpeedMeasurement noisyData;
        noisyData.left = fabs(round(trueData.left + rlDist.getSample()));
        noisyData.right = fabs(round(trueData.right + rrDist.getSample()));
        noisyMeasurements.push_back(std::make_pair(t, noisyData));
        t += T;
      }
    }

    void simulateFrontWheelsSpeedMeasurements(const SplineTrajectory&
        trajectory, double frequency, double sigma2_fl, double sigma2_fr, double
        e_f, double L, double k_fl, double k_fr, const
        sm::kinematics::Transformation& T_io,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type& trueMeasurements,
        MeasurementsContainer<WheelsSpeedMeasurement>::Type&
        noisyMeasurements) {
      const sm::timing::NsecTime trajT_min = trajectory.minTime();
      const sm::timing::NsecTime trajT_max = trajectory.maxTime();
      const NormalDistribution<1> timeDist(0, 3e4);
      const sm::timing::NsecTime odoT_min = trajT_min +
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime odoT_max = trajT_max -
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime T = round(1.0 / frequency * 1e9);
      const NormalDistribution<1> flDist(0, sigma2_fl);
      const NormalDistribution<1> frDist(0, sigma2_fr);
      sm::timing::NsecTime t = odoT_min;
      while (t < odoT_max) {
        const Eigen::Matrix3d C_wi = trajectory.C(t);
        const Eigen::Vector3d v_ii = C_wi.transpose() *
          trajectory.linearVelocity(t);
        const Eigen::Vector3d om_ii = C_wi.transpose() *
          trajectory.angularVelocity(t);
        const Eigen::Vector3d v_oo = T_io.C().transpose() *
          (v_ii + om_ii.cross(T_io.t()));
        const Eigen::Vector3d om_oo = T_io.C().transpose() * om_ii;
        const double v_oo_x = v_oo(0);
        const double om_oo_z = om_oo(2);
        const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
        const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
        if (fabs(cos(phi_L)) > std::numeric_limits<double>::epsilon() ||
            fabs(cos(phi_R) > std::numeric_limits<double>::epsilon())) {
          WheelsSpeedMeasurement trueData;
          trueData.left = fabs(round((v_oo_x - e_f * om_oo_z) / cos(phi_L) /
            k_fl));
          trueData.right = fabs(round((v_oo_x + e_f * om_oo_z) / cos(phi_R) /
            k_fr));
          trueMeasurements.push_back(std::make_pair(t, trueData));
          WheelsSpeedMeasurement noisyData;
          noisyData.left = fabs(round(trueData.left + flDist.getSample()));
          noisyData.right = fabs(round(trueData.right + frDist.getSample()));
          noisyMeasurements.push_back(std::make_pair(t, noisyData));
        }
        t += T;
      }
    }

    void simulateSteeringMeasurements(const SplineTrajectory& trajectory, double
        frequency, double sigma2_st, double L, double a0, double a1, double a2,
        double a3, const sm::kinematics::Transformation&
        T_io, MeasurementsContainer<SteeringMeasurement>::Type&
        trueMeasurements, MeasurementsContainer<SteeringMeasurement>::Type&
        noisyMeasurements) {
      const sm::timing::NsecTime trajT_min = trajectory.minTime();
      const sm::timing::NsecTime trajT_max = trajectory.maxTime();
      const NormalDistribution<1> timeDist(0, 3e4);
      const sm::timing::NsecTime odoT_min = trajT_min +
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime odoT_max = trajT_max -
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime T = round(1.0 / frequency * 1e9);
      const NormalDistribution<1> stDist(0, sigma2_st);
      sm::timing::NsecTime t = odoT_min;
      while (t < odoT_max) {
        const Eigen::Matrix3d C_wi = trajectory.C(t);
        const Eigen::Vector3d v_ii = C_wi.transpose() *
          trajectory.linearVelocity(t);
        const Eigen::Vector3d om_ii = C_wi.transpose() *
          trajectory.angularVelocity(t);
        const Eigen::Vector3d v_oo = T_io.C().transpose() *
          (v_ii + om_ii.cross(T_io.t()));
        const Eigen::Vector3d om_oo = T_io.C().transpose() * om_ii;
        const double v_oo_x = v_oo(0);
        const double om_oo_z = om_oo(2);
        if (std::fabs(v_oo_x) > 1e-1) {
          const double phi = atan(L * om_oo_z / v_oo_x);
          SteeringMeasurement trueData;
          trueData.value = round((phi - a0) / a1);
          trueMeasurements.push_back(std::make_pair(t, trueData));
          SteeringMeasurement noisyData;
          noisyData.value = round(trueData.value + stDist.getSample());
          noisyMeasurements.push_back(std::make_pair(t, noisyData));
        }
        t += T;
      }
    }

    void simulateDMIMeasurements(const SplineTrajectory& trajectory, double
        frequency, double sigma2_dmi, double e_r, const
        sm::kinematics::Transformation& T_io,
        MeasurementsContainer<ApplanixDMIMeasurement>::Type& trueMeasurements,
        MeasurementsContainer<ApplanixDMIMeasurement>::Type&
        noisyMeasurements) {
      const sm::timing::NsecTime trajT_min = trajectory.minTime();
      const sm::timing::NsecTime trajT_max = trajectory.maxTime();
      const NormalDistribution<1> timeDist(0, 3e4);
      const sm::timing::NsecTime odoT_min = trajT_min +
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime odoT_max = trajT_max -
        fabs(round(timeDist.getSample()));
      const sm::timing::NsecTime T = round(1.0 / frequency * 1e9);
      const NormalDistribution<1> dmiDist(0, sigma2_dmi);
      sm::timing::NsecTime t = odoT_min;
      double trueTraveledDistance = 0;
      double noisyTraveledDistance = 0;
      Eigen::Matrix4d T_wi_km1;
      const sm::kinematics::EulerAnglesYawPitchRoll ypr;
      while (t < odoT_max) {
        const Eigen::Matrix4d T_wi_k = trajectory.T(t).T();
        if (t > odoT_min) {
          const Eigen::Matrix4d T_o_km1_o_k = T_io.T().inverse() *
            T_wi_km1.inverse() * T_wi_k * T_io.T();
          const Eigen::Vector3d t_o_km1_o_k = sm::kinematics::transform2rho(
            T_o_km1_o_k);
          const Eigen::Matrix3d C_o_km1_o_k = sm::kinematics::transform2C(
            T_o_km1_o_k);
          const double v_oo_x = t_o_km1_o_k(0);
          const double om_oo_z =
            (ypr.rotationMatrixToParameters(C_o_km1_o_k))(0);
          const double trueDisplacement = (v_oo_x - e_r * om_oo_z);
          const double noisyDisplacement = trueDisplacement +
            dmiDist.getSample();
          trueTraveledDistance += trueDisplacement;
          noisyTraveledDistance += noisyDisplacement;
          ApplanixDMIMeasurement trueData;
          trueData.signedDistanceTraveled = trueTraveledDistance;
          trueMeasurements.push_back(std::make_pair(t, trueData));
          ApplanixDMIMeasurement noisyData;
          noisyData.signedDistanceTraveled = noisyTraveledDistance;
          noisyMeasurements.push_back(std::make_pair(t, noisyData));
        }
        t += T;
        T_wi_km1 = T_wi_k;
      }
    }

  }
}
