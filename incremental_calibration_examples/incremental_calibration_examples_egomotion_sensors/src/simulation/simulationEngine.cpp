/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/egomotion/simulation/simulationEngine.h"

#include <cmath>

#include <utility>
#include <vector>
#include <limits>
#include <unordered_map>

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
      const auto type = params.trajectoryParams.type;
      const auto f = params.trajectoryParams.f;
      const auto dt = params.dt;
      const auto A = params.trajectoryParams.A;
      const auto T = params.T;

      NsecTime timestamp = 0;
      std::vector<NsecTime> timestamps;
      timestamps.push_back(timestamp);
      std::vector<Eigen::Vector3d> transPoses;
      transPoses.push_back(params.trajectoryParams.w_T_v_0.t());
      std::vector<Eigen::Vector4d> rotPoses;
      rotPoses.push_back(params.trajectoryParams.w_T_v_0.q());

      if (type == "combined") {
        // straight
        double w_phi_v_km1 = 0;
        Eigen::Vector2d w_r_wv_km1 = Eigen::Vector2d::Zero();
        for (double t = dt; t < T / 3; t += dt) {
          Eigen::Vector2d v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1,
            w_r_wv_km1, t, dt, 0.0, f);
          Eigen::Vector3d v_v_wv_k(v_v_om_wv_k(0), 0.0, 0.0);
          Eigen::Vector3d v_om_wv_k(0.0, 0.0, v_v_om_wv_k(1));
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

        // sinePlanar
        w_phi_v_km1 = std::atan(2 * M_PI * f * A * cos(2 * M_PI * f * dt));
        w_r_wv_km1 = Eigen::Vector2d::Zero();
        auto lastTimestamp = nsecToSec(timestamps.back());
        for (double t = lastTimestamp + dt; t < 2.0 / 3.0 * T; t += dt) {
          Eigen::Vector2d v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1,
            w_r_wv_km1, t, dt, A, f);
          Eigen::Vector3d v_v_wv_k(v_v_om_wv_k(0), 0.0, 0.0);
          Eigen::Vector3d v_om_wv_k(0.0, 0.0, v_v_om_wv_k(1));
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

        // random
        w_phi_v_km1 = std::atan(2 * M_PI * f * A * cos(2 * M_PI * f * dt));
        w_r_wv_km1 = Eigen::Vector2d::Zero();
        lastTimestamp = nsecToSec(timestamps.back());
        auto trajDist = NormalDistribution<3>(
          Eigen::Matrix<double, 3, 1>::Zero(),
          Eigen::Matrix<double, 3, 3>::Identity() * 2);
        for (double t = lastTimestamp + dt; t < T; t += dt) {
          Eigen::Vector2d v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1,
            w_r_wv_km1, t, dt, A, f);
          auto sample = trajDist.getSample();
          Eigen::Vector3d v_v_wv_k(v_v_om_wv_k(0), v_v_om_wv_k(0),
            v_v_om_wv_k(0));
          v_v_wv_k = v_v_wv_k + sample;
          sample = trajDist.getSample();
          Eigen::Vector3d v_om_wv_k(v_v_om_wv_k(1), v_v_om_wv_k(1),
            v_v_om_wv_k(1));
          v_om_wv_k = v_om_wv_k + sample;
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
      }
      else {
        double w_phi_v_km1;
        if (type == "straight")
          w_phi_v_km1 = 0;
        else
          w_phi_v_km1 = std::atan(2 * M_PI * f * A * cos(2 * M_PI * f * dt));
        Eigen::Vector2d w_r_wv_km1 = Eigen::Vector2d::Zero();

        auto trajDist = NormalDistribution<3>(
          Eigen::Matrix<double, 3, 1>::Zero(),
          Eigen::Matrix<double, 3, 3>::Identity() * 2);

        // generate trajectory
        for (double t = dt; t < T; t += dt) {
          Eigen::Vector2d v_v_om_wv_k;
          if (type == "straight")
            v_v_om_wv_k =
              genSineBodyVel2d(w_phi_v_km1, w_r_wv_km1, t, dt, 0.0, f);
          else
            v_v_om_wv_k = genSineBodyVel2d(w_phi_v_km1, w_r_wv_km1, t, dt, A,
              f);
          Eigen::Vector3d v_v_wv_k;
          Eigen::Vector3d v_om_wv_k;
          if (type == "random") {
            auto sample = trajDist.getSample();
            v_v_wv_k = Eigen::Vector3d(v_v_om_wv_k(0), v_v_om_wv_k(0),
              v_v_om_wv_k(0));
            v_v_wv_k = v_v_wv_k + sample;
            sample = trajDist.getSample();
            v_om_wv_k = Eigen::Vector3d(v_v_om_wv_k(1), v_v_om_wv_k(1),
              v_v_om_wv_k(1));
            v_om_wv_k = v_om_wv_k + sample;
          }
          else {
            v_v_wv_k = Eigen::Vector3d(v_v_om_wv_k(0), 0.0, 0.0);
            v_om_wv_k = Eigen::Vector3d(0.0, 0.0, v_v_om_wv_k(1));
          }
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
      }

      // fit splines
      const auto elapsedTime = (timestamps.back() - timestamps.front()) /
        static_cast<double>(NsecTimePolicy::getOne());
      const auto numMeasurements = timestamps.size();
      const auto measPerSec = std::lround(numMeasurements / elapsedTime);
      int numSegments;
      if (measPerSec > params.trajectoryParams.splineKnotsPerSecond)
        numSegments = std::ceil(params.trajectoryParams.splineKnotsPerSecond *
          elapsedTime);
      else
        numSegments = numMeasurements;

      data.trajectory.translationSpline =
        boost::make_shared<Trajectory::TranslationSpline>(
        EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::CONF(
        EuclideanBSpline<Eigen::Dynamic, 3,
        NsecTimePolicy>::CONF::ManifoldConf(3),
        params.trajectoryParams.transSplineOrder));
      BSplineFitter<Trajectory::TranslationSpline>::initUniformSpline(
        *data.trajectory.translationSpline, timestamps, transPoses, numSegments,
        params.trajectoryParams.transSplineLambda);

      data.trajectory.rotationSpline =
        boost::make_shared<Trajectory::RotationSpline>(
        UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::CONF(
        UnitQuaternionBSpline<Eigen::Dynamic,
        NsecTimePolicy>::CONF::ManifoldConf(),
        params.trajectoryParams.rotSplineOrder));
      BSplineFitter<Trajectory::RotationSpline>::initUniformSpline(
        *data.trajectory.rotationSpline, timestamps, rotPoses, numSegments,
        params.trajectoryParams.rotSplineLambda);

      // generate sensor data
      auto prevTransformation = Transformation();
      const auto referenceSensor = params.referenceSensor;
      bool firstTime = true;
      std::unordered_map<size_t, NormalDistribution<6> > normDists;
      for (const auto& cov : params.sigma2)
        normDists[cov.first] = NormalDistribution<6>(
          Eigen::Matrix<double, 6, 1>::Zero(), cov.second);
      for (auto t = data.trajectory.translationSpline->getMinTime();
          t <= data.trajectory.translationSpline->getMaxTime();
          t += secToNsec(dt)) {
        auto translationEvaluator =
          data.trajectory.translationSpline->getEvaluatorAt<0>(t);
        auto rotationEvaluator =
          data.trajectory.rotationSpline->getEvaluatorAt<0>(t);
        const auto currentTransformation =
          Transformation(rotationEvaluator.eval(), translationEvaluator.eval());
        if (!firstTime) {
          for (const auto& cov : params.sigma2) {
            if (cov.first == referenceSensor) {
              auto w_T_s = prevTransformation.inverse() * currentTransformation;
              MotionMeasurement motion;
              motion.motion = w_T_s;
              motion.duration = secToNsec(dt);
              motion.sigma2 = cov.second;
              data.motionData[cov.first].push_back(std::make_pair(t, motion));
              auto normSample = normDists[cov.first].getSample();
              motion.motion = w_T_s * Transformation(qexp(normSample.tail<3>()),
                normSample.head<3>());
              data.motionDataNoisy[cov.first].push_back(std::make_pair(t,
                motion));
            }
            else {
              auto r_T_s = params.sensorCalibration.at(cov.first).second;
              auto timeDelay = params.sensorCalibration.at(cov.first).first;
              auto w_T_s = r_T_s.inverse() * prevTransformation.inverse() *
                currentTransformation * r_T_s;
              MotionMeasurement motion;
              motion.motion = w_T_s;
              motion.duration = secToNsec(dt);
              motion.sigma2 = cov.second;
              data.motionData[cov.first].push_back(std::make_pair(t + timeDelay,
                motion));
              auto normSample = normDists[cov.first].getSample();
              motion.motion = w_T_s * Transformation(qexp(normSample.tail<3>()),
                normSample.head<3>());
              data.motionDataNoisy[cov.first].push_back(std::make_pair(t +
                timeDelay, motion));
            }
          }
        }
        prevTransformation = currentTransformation;
        firstTime = false;
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

  }
}
