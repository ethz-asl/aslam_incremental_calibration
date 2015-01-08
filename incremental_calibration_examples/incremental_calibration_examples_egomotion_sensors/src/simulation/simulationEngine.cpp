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

#include "aslam/calibration/egomotion/simulation/simulationEngine.h"

#include <cmath>

#include <utility>
#include <vector>
#include <limits>

#include <boost/make_shared.hpp>

#include <bsplines/BSplineFitter.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/egomotion/simulation/Trajectory.h"
#include "aslam/calibration/egomotion/simulation/SimulationData.h"
#include "aslam/calibration/egomotion/simulation/SimulationParams.h"
#include "aslam/calibration/egomotion/algo/bestQuat.h"

using namespace sm::timing;
using namespace sm::kinematics;
using namespace bsplines;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void simulate(const SimulationParams& params, SimulationData& data) {
      const auto f = params.trajectoryParams.f;
      const auto dt = params.dt;
      const auto A = params.trajectoryParams.A;
      const auto T = params.T;
      double w_phi_v_km1 = std::atan(2 * M_PI * f * A * cos(2 * M_PI * f * dt));
      Eigen::Vector2d w_r_wv_km1 = Eigen::Vector2d::Zero();
      NsecTime timestamp = 0;

      std::vector<NsecTime> timestamps;
      timestamps.push_back(timestamp);
      std::vector<Eigen::Vector3d> transPoses;
      transPoses.push_back(params.trajectoryParams.w_T_v_0.t());
      std::vector<Eigen::Vector4d> rotPoses;
      rotPoses.push_back(params.trajectoryParams.w_T_v_0.q());

      // generate trajectory
      for (double t = dt; t < T; t += dt) {
        const Eigen::Vector2d v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1,
          w_r_wv_km1, t, dt, A, f);
        const Eigen::Vector3d v_v_wv_k(v_v_om_wv_k(0), 0.0, 0.0);
        const Eigen::Vector3d v_om_wv_k(0.0, 0.0, v_v_om_wv_k(1));
        const auto w_T_v_t = integrateMotionModel(
          Transformation(rotPoses.back(), transPoses.back()), v_v_wv_k,
          v_om_wv_k, dt);
        timestamp += secToNsec(dt);
        timestamps.push_back(timestamp);
        transPoses.push_back(w_T_v_t.t());
        auto q = w_T_v_t.q();
        if (!rotPoses.empty())
          q = bestQuat(rotPoses.back(), q);
        rotPoses.push_back(q);
      }

      // fit splines
      const auto elapsedTime = (timestamps.back() - timestamps.front()) /
        static_cast<double>(NsecTimePolicy::getOne());
      const auto numMeasurements = timestamps.size();
      const auto measPerSec = std::lround(numMeasurements / elapsedTime);
      int numSegments;
      if (measPerSec > 5) // PARAM
        numSegments = std::ceil(5 * elapsedTime);
      else
        numSegments = numMeasurements;

      data.trajectory.translationSpline =
        boost::make_shared<Trajectory::TranslationSpline>(
        EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::CONF(
        EuclideanBSpline<Eigen::Dynamic, 3,
        NsecTimePolicy>::CONF::ManifoldConf(3), 4)); // PARAM
      BSplineFitter<Trajectory::TranslationSpline>::initUniformSpline(
        *data.trajectory.translationSpline, timestamps, transPoses, numSegments,
        1e-3); // PARAM

      data.trajectory.rotationSpline =
        boost::make_shared<Trajectory::RotationSpline>(
        UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::CONF(
        UnitQuaternionBSpline<Eigen::Dynamic,
        NsecTimePolicy>::CONF::ManifoldConf(), 4)); // PARAM
      BSplineFitter<Trajectory::RotationSpline>::initUniformSpline(
        *data.trajectory.rotationSpline, timestamps, rotPoses, numSegments,
        1e-3); // PARAM

      // generate sensor data
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

  }
}
