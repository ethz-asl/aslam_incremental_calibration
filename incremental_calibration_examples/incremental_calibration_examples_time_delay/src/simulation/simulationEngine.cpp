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

#include "aslam/calibration/time-delay/simulation/simulationEngine.h"

#include <cmath>

#include <utility>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/time-delay/simulation/SimulationData.h"
#include "aslam/calibration/time-delay/simulation/SimulationParams.h"
#include "aslam/calibration/time-delay/data/WheelSpeedMeasurement.h"
#include "aslam/calibration/time-delay/data/PoseMeasurement.h"

using namespace sm::timing;
using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void simulate(const SimulationParams& params, SimulationData& data) {
      data.trajectory.w_T_v.push_back(params.trajectoryParams.w_T_v_0);
      data.trajectory.v_v_wv.push_back(Eigen::Vector3d::Zero());
      data.trajectory.v_om_wv.push_back(Eigen::Vector3d::Zero());
      const double f = params.trajectoryParams.f;
      const double dt = params.dt;
      const double A = params.trajectoryParams.A;
      const double T = params.T;
      double w_phi_v_km1 = std::atan(2 * M_PI * f * A * cos(2 * M_PI * f * dt));
      Eigen::Vector2d w_r_wv_km1 = Eigen::Vector2d::Zero();
      NsecTime timestamp = 0;
      for (double t = dt; t < T; t += dt) {
        // trajectory
        const Eigen::Vector2d v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1,
          w_r_wv_km1, t, dt, A, f);
        const Eigen::Vector3d v_v_wv_k(v_v_om_wv_k(0), 0.0, 0.0);
        const Eigen::Vector3d v_om_wv_k(0.0, 0.0, v_v_om_wv_k(1));
        data.trajectory.w_T_v.push_back(integrateMotionModel(
          data.trajectory.w_T_v.back(), v_v_wv_k, v_om_wv_k, dt));
        data.trajectory.v_v_wv.push_back(v_v_wv_k);
        data.trajectory.v_om_wv.push_back(v_om_wv_k);

        // wheel speed sensors
        data.w_v_wwl.push_back(genWheelVelocity(v_v_wv_k, v_om_wv_k,
          Eigen::Vector3d(0.0, params.b, 0.0)));
        data.w_v_wwr.push_back(genWheelVelocity(v_v_wv_k, v_om_wv_k,
          Eigen::Vector3d(0.0, -params.b, 0.0)));

        WheelSpeedMeasurement lw;
        lw.value = params.k_l * data.w_v_wwl.back()(0);
        data.lwData.push_back(std::make_pair(timestamp + secToNsec(params.t_l),
          lw));

        const NormalDistribution<1> lwDist(0, params.sigma2_l);
        lw.value = params.k_l * (data.w_v_wwl.back()(0) + lwDist.getSample());
        data.lwData_n.push_back(std::make_pair(timestamp +
          secToNsec(params.t_l), lw));

        WheelSpeedMeasurement rw;
        rw.value = params.k_r * data.w_v_wwr.back()(0);
        data.rwData.push_back(std::make_pair(timestamp + secToNsec(params.t_r),
          rw));

        const NormalDistribution<1> rwDist(0, params.sigma2_r);
        rw.value = params.k_r * (data.w_v_wwr.back()(0) + rwDist.getSample());
        data.rwData_n.push_back(std::make_pair(timestamp +
          secToNsec(params.t_r), rw));

        // pose sensor
        PoseMeasurement pose;
        const EulerAnglesYawPitchRoll ypr;
        pose.w_r_wp = data.trajectory.w_T_v.back().t() +
          data.trajectory.w_T_v.back().C() * params.v_T_p.t();
        pose.sigma2_w_r_wp = params.sigma2_w_r_wp;
        pose.w_R_p = ypr.rotationMatrixToParameters(
          data.trajectory.w_T_v.back().C() * params.v_T_p.C());
        pose.sigma2_w_R_p = params.sigma2_w_R_p;
        data.poseData.push_back(std::make_pair(timestamp, pose));

        const NormalDistribution<3> w_r_wpDist(Eigen::Vector3d::Zero(),
          pose.sigma2_w_r_wp);
        pose.w_r_wp = pose.w_r_wp + w_r_wpDist.getSample();
        const NormalDistribution<3>
          w_R_pDist(Eigen::Vector3d::Zero(), pose.sigma2_w_R_p);
        pose.w_R_p = pose.w_R_p + w_R_pDist.getSample();
        data.poseData_n.push_back(std::make_pair(timestamp, pose));

        timestamp += secToNsec(dt);
      }
    }

    Transformation integrateMotionModel(const Transformation& w_T_v_km1, const
        Eigen::Vector3d& v_v_wv_km1, const Eigen::Vector3d& v_om_wv_km1, double
        dt) {
      const Eigen::Vector4d w_q_v_km1 = w_T_v_km1.q();
      const Eigen::Vector3d w_r_wv_k = w_T_v_km1.t() + dt *
        quatRotate(w_q_v_km1, v_v_wv_km1);
      Eigen::Vector4d v_q_w_k = quatInv(w_q_v_km1);
      const double v_om_wv_km1_norm2 = v_om_wv_km1.norm();
      if (v_om_wv_km1_norm2 > std::numeric_limits<double>::epsilon()) {
        const double c = std::cos(v_om_wv_km1_norm2 / 2.0 * dt);
        const double s = std::sin(v_om_wv_km1_norm2 / 2.0 * dt);
        Eigen::Vector4d sigma;
        sigma.head<3>() = v_om_wv_km1 / v_om_wv_km1_norm2 * s;
        sigma(3) = c;
        v_q_w_k = qplus(sigma, v_q_w_k);
      }

      return Transformation(quatInv(v_q_w_k), w_r_wv_k);
    }

    Eigen::Vector2d genSineBodyVel2d(double& w_phi_v_km1, Eigen::Vector2d&
        w_r_wv_km1, double t, double dt, double A, double f) {
      const double w_phi_v_k =
        std::atan(2.0 * M_PI * f * A * cos(2 * M_PI * f * t));
      const double v_om_wv_k = (w_phi_v_k - w_phi_v_km1) / dt;
      const Eigen::Vector2d w_r_wv_k(t, A * sin(2.0 * M_PI * f * t));
      const Eigen::Vector2d w_r_v_k_km1v_k = w_r_wv_k - w_r_wv_km1;
      const Eigen::Vector2d w_v_wv_k = w_r_v_k_km1v_k / dt;
      const double c = std::cos(w_phi_v_k);
      const double s = std::sin(w_phi_v_k);
      Eigen::Matrix2d w_R_v_k;
      w_R_v_k << c, -s, s, c;
      Eigen::Vector2d v_v_wv_k = w_R_v_k.transpose() * w_v_wv_k;
      w_phi_v_km1 = w_phi_v_k;
      w_r_wv_km1 = w_r_wv_k;
      return Eigen::Vector2d(v_v_wv_k(0), v_om_wv_k);
    }

    Eigen::Vector3d genWheelVelocity(const Eigen::Vector3d& v_v_wv, const
        Eigen::Vector3d& v_om_wv, const Eigen::Vector3d& v_r_vc) {
      return v_v_wv + v_om_wv.cross(v_r_vc);
    }


  }
}
