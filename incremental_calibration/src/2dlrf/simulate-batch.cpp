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

/** \file simulate-batch.cpp
    \brief This file runs a simulation of the calibration problem in batch mode.
  */

#include <iostream>
#include <fstream>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/three_point_methods.hpp>

#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/Optimizer2.hpp>

#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/statistics/UniformDistribution.h"
#include "aslam/calibration/statistics/NormalDistribution.h"
#include "aslam/calibration/2dlrf/utils.h"
#include "aslam/calibration/data-structures/VectorDesignVariable.h"
#include "aslam/calibration/2dlrf/ErrorTermMotion.h"
#include "aslam/calibration/2dlrf/ErrorTermObservation.h"
#include "aslam/calibration/geometry/Transformation.h"
#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/algorithms/matrixOperations.h"

using namespace aslam::calibration;
using namespace aslam::backend;
using namespace sm::kinematics;

int main(int argc, char** argv) {

  // steps to simulate
  const size_t steps = 5000;

  // timestep size
  const double T = 0.1;

  // true state
  std::vector<Eigen::Vector3d > x_true;
  x_true.reserve(steps);

  // integrated odometry
  std::vector<Eigen::Vector3d > x_odom;
  x_odom.reserve(steps);

  // true control input
  std::vector<Eigen::Vector3d > u_true;
  const double sineWaveAmplitude = 0.0;
  const double sineWaveFrequency = 0.01;
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);

  // measured control input
  std::vector<Eigen::Vector3d > u_noise;
  u_noise.reserve(steps);

  // number of landmarks
  const size_t nl = 17;

  // playground size
  const Eigen::Vector2d min(0, 0);
  const Eigen::Vector2d max(30, 30);

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
  Q(0, 0) = 0.00044; Q(1, 1) = 1e-6; Q(2, 2) = 0.00082;

  // covariance matrix for observation model
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = 0.00090; R(1, 1) = 0.00067;

  // landmark positions
  std::vector<Eigen::Vector2d > x_l;
  UniformDistribution<double, 2>(min, max).getSamples(x_l, nl);

  // true calibration parameters
  const Eigen::Vector3d Theta(0.219, 0.1, 0.78);

  // guessed calibration parameters
  const Eigen::Vector3d Theta_hat(0.23, 0.11, 0.8);

  // initial state
  const Eigen::Vector3d x_0(1.0, 1.0, M_PI / 4);
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
    xk(3) = angleMod(xk(3));
    x_true.push_back(xk);
    u_noise.push_back(u_true[i] +
      NormalDistribution<3>(Eigen::Vector3d::Zero(), Q).getSample());
    B(0, 0) = cos(x_odom[i - 1](2));
    B(0, 1) = -sin(x_odom[i - 1](2));
    B(1, 0) = sin(x_odom[i - 1](2));
    B(1, 1) = cos(x_odom[i - 1](2));
    xk = x_odom[i - 1] + T * B * u_noise[i];
    xk(3) = angleMod(xk(3));
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
        NormalDistribution<1>(0, R(0, 0)).getSample();
      rk[j] = range;
      bk[j] = angleMod(atan2(bb, aa) - x_true[i](2) - Theta(2) +
        NormalDistribution<1>(0, R(1, 1)).getSample());
    }
    r.push_back(rk);
    b.push_back(bk);
  }

  // landmark guess
  std::vector<Eigen::Vector2d > x_l_hat;
  initLandmarks(x_l_hat, x_odom, Theta_hat, r, b);

  // create optimization problem
  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem);

  // create calibration parameters design variable
  boost::shared_ptr<VectorDesignVariable<3> >
    dv_Theta(new VectorDesignVariable<3>(Theta_hat));
  dv_Theta->setActive(true);
  problem->addDesignVariable(dv_Theta, 2);

  // create state design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<3> > > dv_x;
  dv_x.reserve(steps);
  for (size_t i = 0; i < steps; ++i) {
    dv_x.push_back(boost::shared_ptr<VectorDesignVariable<3> >
      (new VectorDesignVariable<3>(x_odom[i])));
    dv_x[i]->setActive(true);
    problem->addDesignVariable(dv_x[i], 0);
  }

  // create landmarks design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<2> > > dv_x_l;
  dv_x_l.reserve(nl);
  for (size_t i = 0; i < nl; ++i) {
    dv_x_l.push_back(boost::shared_ptr<VectorDesignVariable<2> >
      (new VectorDesignVariable<2>(x_l_hat[i])));
    dv_x_l[i]->setActive(true);
    problem->addDesignVariable(dv_x_l[i], 1);
  }

  // set the ordering of the problem
  problem->setGroupsOrdering({0, 1, 2});

  // add motion and observation error terms
  for (size_t i = 1; i < steps; ++i) {
    boost::shared_ptr<ErrorTermMotion> e_mot(
      new ErrorTermMotion(dv_x[i - 1].get(), dv_x[i].get(), T, u_noise[i], Q));
    problem->addErrorTerm(e_mot);
    for (size_t j = 0; j < nl; ++j) {
      boost::shared_ptr<ErrorTermObservation> e_obs(
        new ErrorTermObservation(dv_x[i].get(), dv_x_l[j].get(), dv_Theta.get(),
        r[i][j], b[i][j], R));
      problem->addErrorTerm(e_obs);
    }
  }

  std::cout << "Calibration before: " << *dv_Theta << std::endl;

  Optimizer2Options options;
  options.verbose = true;
  options.doLevenbergMarquardt = false;
  options.linearSolver = "sparse_qr";
  SparseQRLinearSolverOptions linearSolverOptions;
  linearSolverOptions.colNorm = true;
  linearSolverOptions.qrTol = 0.02;
  Optimizer2 optimizer(options);
  optimizer.getSolver<SparseQrLinearSystemSolver>()
    ->setOptions(linearSolverOptions);
  optimizer.setProblem(problem);
  const double before = Timestamp::now();
  optimizer.optimize();
  const double after = Timestamp::now();
  std::cout << "Elapsed time [s]: " << after - before << std::endl;
  std::cout << "Calibration after: " << *dv_Theta << std::endl;
  const CompressedColumnMatrix<ssize_t>& RFactor =
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getR();
  const size_t numCols = RFactor.cols();
  std::cout << "Sigma: " << std::endl
    << computeCovariance(RFactor, numCols - dv_Theta->minimalDimensions(),
    numCols - 1) << std::endl;

  // output results to file
  std::ofstream x_true_log("x_true.txt");
  for (size_t i = 0; i < steps; ++i)
    x_true_log << x_true[i].transpose() << std::endl;
  std::ofstream x_odom_log("x_odom.txt");
  for (size_t i = 0; i < steps; ++i)
    x_odom_log << x_odom[i].transpose() << std::endl;
  std::ofstream x_est_log("x_est.txt");
  for (size_t i = 0; i < steps; ++i)
    x_est_log << *(dv_x[i]) << std::endl;
  std::ofstream l_log("l.txt");
  for (size_t i = 0; i < nl; ++i)
    l_log << x_l[i].transpose() << std::endl;
  std::ofstream l_est_log("l_est.txt");
  for (size_t i = 0; i < nl; ++i)
    l_est_log << *(dv_x_l[i]) << std::endl;

  // align landmarks
  Eigen::MatrixXd l = Eigen::MatrixXd::Zero(3, nl);
  Eigen::MatrixXd l_est = Eigen::MatrixXd::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i) {
    l(0, i) = x_l[i](0);
    l(1, i) = x_l[i](1);
    l_est(0, i) = dv_x_l[i]->getValue()(0);
    l_est(1, i) = dv_x_l[i]->getValue()(1);
  }
  Transformation<double, 3> trans(sm::kinematics::threePointSvd(l, l_est));
  Eigen::MatrixXd l_est_trans = Eigen::MatrixXd::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i)
    l_est_trans.col(i) = trans(l_est.col(i));
  std::ofstream l_est_trans_log("l_est_trans.txt");
  for (size_t i = 0; i < nl; ++i)
    l_est_trans_log << l_est_trans.col(i).head<2>().transpose() << std::endl;

  // align poses
  std::vector<Eigen::Vector3d > x_est_trans;
  x_est_trans.reserve(steps);
  std::ofstream x_est_trans_log("x_est_trans.txt");
  for (size_t i = 0; i < steps; ++i) {
    Eigen::Vector3d pose((Eigen::Vector3d()
      << dv_x[i]->getValue().head<2>(), 0).finished());
    trans.transform(pose, pose);
    pose(2) = dv_x[i]->getValue()(2);
    x_est_trans.push_back(pose);
    x_est_trans_log << x_est_trans[i].transpose() << std::endl;
  }

  return 0;
}
