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

/** \file simulate-it-new.cpp
    \brief This file runs a simulation of the calibration problem in iterative
           mode.
  */

#include <vector>

#include <Eigen/Core>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/three_point_methods.hpp>

#include "aslam/calibration/statistics/UniformDistribution.h"
#include "aslam/calibration/statistics/NormalDistribution.h"
#include "aslam/calibration/2dlrf/utils.h"
#include "aslam/calibration/data-structures/VectorDesignVariable.h"
#include "aslam/calibration/2dlrf/ErrorTermMotion.h"
#include "aslam/calibration/2dlrf/ErrorTermObservation.h"
#include "aslam/calibration/geometry/Transformation.h"
#include "aslam/calibration/core/IncrementalEstimator.h"

int main(int argc, char** argv) {
  // steps to simulate
  const size_t steps = 5000;

  // timestep size
  const double T = 0.1;

  // true state
  std::vector<Eigen::Vector3d> x_true;
  x_true.reserve(steps);

  // integrated odometry
  std::vector<Eigen::Vector3d> x_odom;
  x_odom.reserve(steps);

  // true control input
  std::vector<Eigen::Vector3d> u_true;
  const double sineWaveAmplitude = 1.0;
  const double sineWaveFrequency = 0.01;
  aslam::calibration::genSineWavePath(u_true, steps, sineWaveAmplitude,
    sineWaveFrequency, T);

  // measured control input
  std::vector<Eigen::Vector3d> u_noise;
  u_noise.reserve(steps);

  // number of landmarks
  const size_t nl = 17;

  // playground size
  Eigen::Vector2d min(0, 0);
  Eigen::Vector2d max(30, 30);

  // bearing measurements
  std::vector<std::vector<double> > b;
  b.reserve(steps);
  b.push_back(std::vector<double>(nl, 0));

  // range measurements
  std::vector<std::vector<double> > r;
  r.reserve(steps);
  r.push_back(std::vector<double>(nl, 0));

  // covariance matrix for motion model
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = 0.00044;
  Q(1, 1) = 1e-6;
  Q(2, 2) = 0.00082;

  // covariance matrix for observation model
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = 0.00090;
  R(1, 1) = 0.00067;

  // landmark positions
  std::vector<Eigen::Vector2d> x_l;
  aslam::calibration::UniformDistribution<double, 2>(min, max).
    getSamples(x_l, nl);

  // true calibration parameters
  Eigen::Vector3d Theta(0.219, 0.1, 0.78);

  // guessed calibration parameters
  Eigen::Vector3d Theta_hat(0.23, 0.11, 0.8);

  // initial state
  Eigen::Vector3d x_0(1.0, 1.0, M_PI / 4);
  x_true.push_back(x_0);
  x_odom.push_back(x_0);
  u_noise.push_back(Eigen::Vector3d::Zero());

  // simulate
  for (size_t i = 1; i < steps; ++i) {
    Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
    B(0, 0) = cos(x_true[i - 1](2));
    B(0, 1) = -sin(x_true[i - 1](2));
    B(1, 0) = sin(x_true[i - 1](2));
    B(1, 1) = cos(x_true[i - 1](2));
    Eigen::Vector3d xk = x_true[i - 1] + T * B * u_true[i];
    xk(3) = sm::kinematics::angleMod(xk(3));
    x_true.push_back(xk);
    u_noise.push_back(u_true[i] +
      aslam::calibration::NormalDistribution<3>(
      Eigen::Vector3d::Zero(), Q).getSample());
    B(0, 0) = cos(x_odom[i - 1](2));
    B(0, 1) = -sin(x_odom[i - 1](2));
    B(1, 0) = sin(x_odom[i - 1](2));
    B(1, 1) = cos(x_odom[i - 1](2));
    xk = x_odom[i - 1] + T * B * u_noise[i];
    xk(3) = sm::kinematics::angleMod(xk(3));
    x_odom.push_back(xk);
    const double ct = cos(x_true[i](2));
    const double st = sin(x_true[i](2));
    std::vector<double> rk(nl, 0);
    std::vector<double> bk(nl, 0);
    for (size_t j = 0; j < nl; ++j) {
      const double aa = x_l[j](0) - x_true[i](0) - Theta(0) * ct +
        Theta(1) * st;
      const double bb = x_l[j](1) - x_true[i](1) - Theta(0) * st -
        Theta(1) * ct;
      const double range = sqrt(aa * aa + bb * bb) +
        aslam::calibration::NormalDistribution<1>(
        0, R(0, 0)).getSample();
      rk[j] = range;
      bk[j] = sm::kinematics::angleMod(atan2(bb, aa) - x_true[i](2)
        - Theta(2) + aslam::calibration::NormalDistribution<1>(
        0, R(1, 1)).getSample());
    }
    r.push_back(rk);
    b.push_back(bk);
  }

  // landmark guess
  std::vector<Eigen::Vector2d> x_l_hat;
  aslam::calibration::initLandmarks(x_l_hat, x_odom, Theta_hat, r, b);

  // create landmarks design variables
  aslam::calibration::IncrementalEstimator::DVContainer dv_x_l;
  dv_x_l.reserve(nl);
  for (size_t i = 0; i < nl; ++i) {
    dv_x_l.push_back(
      boost::shared_ptr<aslam::calibration::VectorDesignVariable<2> >
      (new aslam::calibration::VectorDesignVariable<2>(x_l_hat[i])));
    dv_x_l[i]->setActive(true);
  }

  // create calibration parameters design variable
  boost::shared_ptr<aslam::calibration::VectorDesignVariable<3> >
    dv_Theta(new aslam::calibration::VectorDesignVariable<3>(Theta_hat));
  dv_Theta->setActive(true);

  // design variable to be marginalized
  aslam::calibration::IncrementalEstimator::DVContainer designVariablesMarg;
  designVariablesMarg.push_back(dv_Theta);

  // create incremental estimator
  aslam::calibration::IncrementalEstimator
    incrementalEstimator(designVariablesMarg, dv_x_l);

  // batch size
  const size_t batchSize = 200;

  // run over the dataset
  for (size_t i = 0; i < steps; i += batchSize) {
    // new design variables to be added
    aslam::calibration::IncrementalEstimator::DVContainer designVariablesNew;

    // new error terms to be added
    aslam::calibration::IncrementalEstimator::ETContainer errorTermsNew;

    // create starting state variable, activate it, and push it
    boost::shared_ptr<aslam::calibration::VectorDesignVariable<3> >
      dv_x0(new aslam::calibration::VectorDesignVariable<3>(x_odom[i]));
    dv_x0->setActive(true);
    designVariablesNew.push_back(dv_x0);

    // create other state variables and error terms
    for (size_t j = i + 1; j < i + batchSize && j < steps; ++j) {
      // create state variable, activate it, and push it
      boost::shared_ptr<aslam::calibration::VectorDesignVariable<3> >
        dv_xk(new aslam::calibration::VectorDesignVariable<3>(x_odom[j]));
      dv_xk->setActive(true);
      designVariablesNew.push_back(dv_xk);

      // previous state variable
      const size_t km1 = designVariablesNew.size() - 2;
      aslam::calibration::VectorDesignVariable<3>* dv_xkm1 =
        dynamic_cast<aslam::calibration::VectorDesignVariable<3>*>(
        designVariablesNew[km1].get());

      // motion error term
      boost::shared_ptr<aslam::calibration::ErrorTermMotion> e_mot(
        new aslam::calibration::ErrorTermMotion(dv_xkm1, dv_xk.get(), T,
        u_noise[j], Q));
      errorTermsNew.push_back(e_mot);

      // observation error terms
      for (size_t k = 0; k < nl; ++k) {
        aslam::calibration::VectorDesignVariable<2>* dv_l =
          dynamic_cast<aslam::calibration::VectorDesignVariable<2>*>(
          dv_x_l[k].get());
        boost::shared_ptr<aslam::calibration::ErrorTermObservation> e_obs(
          new aslam::calibration::ErrorTermObservation(dv_xk.get(),
          dv_l, dv_Theta.get(), r[j][k], b[j][k], R));
        errorTermsNew.push_back(e_obs);
      }
    }

    // add the measurement batch to the estimator
    incrementalEstimator.addMeasurementBatch(errorTermsNew, designVariablesNew);
  }

  // output results to file
  std::ofstream x_true_log("x_true.txt");
  for (size_t i = 0; i < steps; ++i)
    x_true_log << x_true[i].transpose() << std::endl;
  std::ofstream x_odom_log("x_odom.txt");
  for (size_t i = 0; i < steps; ++i)
    x_odom_log << x_odom[i].transpose() << std::endl;
  std::ofstream x_est_log("x_est.txt");
  for (auto it = incrementalEstimator.getDVICBegin();
      it != incrementalEstimator.getDVICEnd(); ++ it) {
    aslam::calibration::VectorDesignVariable<3>* dv_xk =
      dynamic_cast<aslam::calibration::VectorDesignVariable<3>*>(it->get());
    x_est_log << *dv_xk << std::endl;
  }
  std::ofstream l_log("l.txt");
  for (size_t i = 0; i < nl; ++i)
    l_log << x_l[i].transpose() << std::endl;
  std::ofstream l_est_log("l_est.txt");
  for (size_t i = 0; i < nl; ++i) {
    aslam::calibration::VectorDesignVariable<2>* dv_l =
      dynamic_cast<aslam::calibration::VectorDesignVariable<2>*>(
      dv_x_l[i].get());
    l_est_log << *(dv_l) << std::endl;
  }

  // align landmarks
  Eigen::MatrixXd l = Eigen::MatrixXd::Zero(3, nl);
  Eigen::MatrixXd l_est = Eigen::MatrixXd::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i) {
    l(0, i) = x_l[i](0);
    l(1, i) = x_l[i](1);
    aslam::calibration::VectorDesignVariable<2>* dv_l =
      dynamic_cast<aslam::calibration::VectorDesignVariable<2>*>(
      dv_x_l[i].get());
    l_est(0, i) = dv_l->getValue()(0);
    l_est(1, i) = dv_l->getValue()(1);
  }
  aslam::calibration::Transformation<double, 3>
    trans(sm::kinematics::threePointSvd(l, l_est));
  Eigen::MatrixXd l_est_trans = Eigen::MatrixXd::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i)
    l_est_trans.col(i) = trans(l_est.col(i));
  std::ofstream l_est_trans_log("l_est_trans.txt");
  for (size_t i = 0; i < nl; ++i)
    l_est_trans_log << l_est_trans.col(i).head<2>().transpose() << std::endl;

  // align poses
  std::vector<Eigen::Vector3d> x_est_trans;
  x_est_trans.reserve(steps);
  std::ofstream x_est_trans_log("x_est_trans.txt");
  for (auto it = incrementalEstimator.getDVICBegin();
      it != incrementalEstimator.getDVICEnd(); ++ it) {
    aslam::calibration::VectorDesignVariable<3>* dv_xk =
      dynamic_cast<aslam::calibration::VectorDesignVariable<3>*>(it->get());
    Eigen::Vector3d pose((Eigen::Vector3d()
      << dv_xk->getValue().head<2>(), 0).finished());
    trans.transform(pose, pose);
    pose(2) = dv_xk->getValue()(2);
    x_est_trans.push_back(pose);
    x_est_trans_log << pose.transpose() << std::endl;
  }

  return 0;
}
