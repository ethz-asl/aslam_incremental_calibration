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

/** \file simulate-online.cpp
    \brief This file runs a simulation of the calibration problem in iterative
           mode.
  */

#include <vector>

#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/three_point_methods.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/Optimizer2.hpp>

#include <aslam/calibration/statistics/UniformDistribution.h>
#include <aslam/calibration/statistics/NormalDistribution.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/geometry/Transformation.h>
#include <aslam/calibration/algorithms/matrixOperations.h>
#include <aslam/calibration/base/Timestamp.h>

#include "aslam/calibration/2dlrf/utils.h"
#include "aslam/calibration/2dlrf/ErrorTermMotion.h"
#include "aslam/calibration/2dlrf/ErrorTermObservation.h"

using namespace aslam::calibration;
using namespace aslam::backend;
using namespace sm::kinematics;

int main(int argc, char** argv) {
  // steps to simulate
  const size_t steps = 5000;

  // timestep size
  const double T = 0.1;

  // true state
  std::vector<Eigen::Matrix<double, 3, 1> > x_true;
  x_true.reserve(steps);

  // integrated odometry
  std::vector<Eigen::Matrix<double, 3, 1> > x_odom;
  x_odom.reserve(steps);

  // true control input
  std::vector<Eigen::Matrix<double, 3, 1> > u_true;
  const double sineWaveAmplitude = 1.0;
  const double sineWaveFrequency = 0.01;
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);

  // measured control input
  std::vector<Eigen::Matrix<double, 3, 1> > u_noise;
  u_noise.reserve(steps);

  // number of landmarks
  const size_t nl = 17;

  // playground size
  Eigen::Matrix<double, 2, 1> min(0, 0);
  Eigen::Matrix<double, 2, 1> max(30, 30);

  // bearing measurements
  std::vector<std::vector<double> > b;
  b.reserve(steps);
  b.push_back(std::vector<double>(nl, 0));

  // range measurements
  std::vector<std::vector<double> > r;
  r.reserve(steps);
  r.push_back(std::vector<double>(nl, 0));

  // covariance matrix for motion model
  Eigen::Matrix<double, 3, 3> Q = Eigen::Matrix<double, 3, 3>::Zero();
  Q(0, 0) = 0.00044;
  Q(1, 1) = 1e-6;
  Q(2, 2) = 0.00082;

  // covariance matrix for observation model
  Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
  R(0, 0) = 0.00090;
  R(1, 1) = 0.00067;

  // landmark positions
  std::vector<Eigen::Matrix<double, 2, 1> > x_l;
  UniformDistribution<double, 2>(min, max).getSamples(x_l, nl);

  // true calibration parameters
  Eigen::Matrix<double, 3, 1> Theta(0.219, 0.1, 0.78);

  // guessed calibration parameters
//  Eigen::Matrix<double, 3, 1> Theta_hat = Theta +
//    aslam::calibration::NormalDistribution<3>(
//    Eigen::Matrix<double, 3, 1>::Zero(),
//    Eigen::Matrix<double, 3, 3>::Identity() * 1e-2).getSample();
  Eigen::Matrix<double, 3, 1> Theta_hat(0.23, 0.11, 0.8);

  // initial state
  Eigen::Matrix<double, 3, 1> x_0(1.0, 1.0, M_PI / 4);
  x_true.push_back(x_0);
  x_odom.push_back(x_0);
  u_noise.push_back(Eigen::Matrix<double, 3, 1>::Zero());

  // simulate
  for (size_t i = 1; i < steps; ++i) {
    Eigen::Matrix<double, 3, 3> B = Eigen::Matrix<double, 3, 3>::Identity();
    B(0, 0) = cos(x_true[i - 1](2));
    B(0, 1) = -sin(x_true[i - 1](2));
    B(1, 0) = sin(x_true[i - 1](2));
    B(1, 1) = cos(x_true[i - 1](2));
    Eigen::Matrix<double, 3, 1> xk = x_true[i - 1] + T * B * u_true[i];
    xk(2) = angleMod(xk(2));
    x_true.push_back(xk);
    u_noise.push_back(u_true[i] + NormalDistribution<3>(
      Eigen::Matrix<double, 3, 1>::Zero(), Q).getSample());
    B(0, 0) = cos(x_odom[i - 1](2));
    B(0, 1) = -sin(x_odom[i - 1](2));
    B(1, 0) = sin(x_odom[i - 1](2));
    B(1, 1) = cos(x_odom[i - 1](2));
    xk = x_odom[i - 1] + T * B * u_noise[i];
    xk(2) = angleMod(xk(2));
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
      const double range = sqrt(aa * aa + bb * bb) + NormalDistribution<1>(
        0, R(0, 0)).getSample();
      rk[j] = range;
      bk[j] = angleMod(atan2(bb, aa) - x_true[i](2) - Theta(2) +
        NormalDistribution<1>(0, R(1, 1)).getSample());
    }
    r.push_back(rk);
    b.push_back(bk);
  }

  // landmark guess
  std::vector<Eigen::Matrix<double, 2, 1> > x_l_hat;
  initLandmarks(x_l_hat, x_odom, Theta_hat, r, b);

  // create state design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<3> > > dv_x;
  dv_x.reserve(steps);
  for (size_t i = 0; i < steps; ++i) {
    dv_x.push_back(
      boost::make_shared<VectorDesignVariable<3> >(x_odom[i]));
    dv_x[i]->setActive(true);
  }

  // create landmarks design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<2> > > dv_x_l;
  dv_x_l.reserve(nl);
  for (size_t i = 0; i < nl; ++i) {
    dv_x_l.push_back(
      boost::make_shared<VectorDesignVariable<2> >(x_l_hat[i]));
    dv_x_l[i]->setActive(true);
  }

  // create calibration parameters design variable
  auto dv_Theta = boost::make_shared<VectorDesignVariable<3> >(Theta_hat);
  dv_Theta->setActive(true);

  // batch size
  const size_t batchSize = 200;

  // mutual information threshold
  const double miTol = 0.5;

  // batch idx used for optimization
  std::vector<size_t> batchIdx;
  batchIdx.reserve(steps / batchSize);

  // calibration parameters record
  Eigen::Matrix <double, 3, 1> calibParams = dv_Theta->getValue();

  // Sigma record
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> SigmaRecord;

  // Sigma determinant record
  double SigmaDetRecord = 0;

  // iterative optimization
  for (size_t i = 0; i < steps; i += batchSize) {
    const double timeStart = Timestamp::now();

    // insert current batch
    batchIdx.push_back(i);

    // create optimization problem
    auto problem = boost::make_shared<OptimizationProblem>();

    // add design variables to the problem
    for (size_t j = 0; j < batchIdx.size(); ++j)
      for (size_t k = batchIdx[j]; k < batchIdx[j] + batchSize; ++k)
        problem->addDesignVariable(dv_x[k]);
    for (size_t j = 0; j < nl; ++j)
      problem->addDesignVariable(dv_x_l[j]);
    problem->addDesignVariable(dv_Theta);

    // add error terms to the problem
    for (size_t j = 0; j < batchIdx.size(); ++j) {
      for (size_t k = batchIdx[j] + 1; k < batchIdx[j] + batchSize; ++k) {
        auto e_mot = boost::make_shared<ErrorTermMotion>(dv_x[k - 1].get(),
          dv_x[k].get(), T, u_noise[k], Q);
        problem->addErrorTerm(e_mot);
        for (size_t l = 0; l < nl; ++l) {
          auto e_obs = boost::make_shared<ErrorTermObservation>(dv_x[k].get(),
            dv_x_l[l].get(), dv_Theta.get(), r[k][l], b[k][l], R);
          problem->addErrorTerm(e_obs);
        }
      }
    }

    std::cout << "Calibration before: " << calibParams.transpose() << std::endl;

    // optimization round
    Optimizer2Options options;
    options.verbose = true;
    options.doLevenbergMarquardt = false;
    options.linearSolver = "sparse_qr";
    SparseQRLinearSolverOptions linearSolverOptions;
    linearSolverOptions.colNorm = true;
    linearSolverOptions.qrTol = 0.02;
    Optimizer2 optimizer(options);
    optimizer.getSolver<SparseQrLinearSystemSolver>()->setOptions(
      linearSolverOptions);
    optimizer.setProblem(problem);
    optimizer.optimize();

    // Sigma computation
    const CompressedColumnMatrix<ssize_t>& RFactor =
      optimizer.getSolver<SparseQrLinearSystemSolver>()->getR();
    const size_t numCols = RFactor.cols();
    Eigen::MatrixXd Sigma = computeCovariance(RFactor,
      numCols - dv_Theta->minimalDimensions(), numCols - 1);
    const double SigmaDet = Sigma.determinant();

    std::cout << "Calibration after: " << *dv_Theta << std::endl;
    std::cout << "Sigma: " << std::endl << Sigma << std::endl;

    // decision round
    if (batchIdx.size() == 1) { // keep if only one batch
      calibParams = dv_Theta->getValue();
      SigmaRecord = Sigma;
      SigmaDetRecord = SigmaDet;
    }
    else {
      // compute mutual information
      const double mi = 0.5 * log2(SigmaDetRecord / SigmaDet);

      // keep batch if needed
      if (mi > miTol) {
        calibParams = dv_Theta->getValue();
        SigmaRecord = Sigma;
        SigmaDetRecord = SigmaDet;
      }
      else
        batchIdx.pop_back();
    }

    const double timeStop = Timestamp::now();
    std::cout << "Batch processing time [s]: " << timeStop - timeStart
      << std::endl;
  }

  std::cout << "Final calibration: " << calibParams.transpose() << std::endl;
  std::cout << "Sigma: " << std::endl << SigmaRecord << std::endl;
  std::cout << "Data used: " << (batchIdx.size() * batchSize) / (double)steps
    * 100 << " %" << std::endl;

  // output results to file
  std::ofstream x_true_log("x_true.txt");
  for (size_t i = 0; i < steps; ++i)
    x_true_log << x_true[i].transpose() << std::endl;
  std::ofstream x_odom_log("x_odom.txt");
  for (size_t i = 0; i < steps; ++i)
    x_odom_log << x_odom[i].transpose() << std::endl;
  std::ofstream x_est_log("x_est.txt");
  for (size_t i = 0; i < batchIdx.size(); ++i)
    for (size_t j = batchIdx[i]; j < batchIdx[i] + batchSize; ++j)
      x_est_log << *(dv_x[j]) << std::endl;
  std::ofstream l_log("l.txt");
  for (size_t i = 0; i < nl; ++i)
    l_log << x_l[i].transpose() << std::endl;
  std::ofstream l_est_log("l_est.txt");
  for (size_t i = 0; i < nl; ++i)
    l_est_log << *(dv_x_l[i]) << std::endl;

  // align landmarks
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> l =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, nl);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> l_est =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i) {
    l(0, i) = x_l[i](0);
    l(1, i) = x_l[i](1);
    l_est(0, i) = dv_x_l[i]->getValue()(0);
    l_est(1, i) = dv_x_l[i]->getValue()(1);
  }
  Transformation<double, 3> trans(threePointSvd(l, l_est));
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> l_est_trans =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(3, nl);
  for (size_t i = 0; i < nl; ++i)
    l_est_trans.col(i) = trans(l_est.col(i));
  std::ofstream l_est_trans_log("l_est_trans.txt");
  for (size_t i = 0; i < nl; ++i)
    l_est_trans_log << l_est_trans.col(i).head<2>().transpose() << std::endl;

  // align poses
  std::vector<Eigen::Matrix<double, 3, 1> > x_est_trans;
  x_est_trans.reserve(steps);
  std::ofstream x_est_trans_log("x_est_trans.txt");
  for (size_t i = 0; i < batchIdx.size(); ++i)
    for (size_t j = batchIdx[i]; j < batchIdx[i] + batchSize; ++j) {
      Eigen::Matrix<double, 3, 1> pose((Eigen::Matrix<double, 3, 1>()
        << dv_x[j]->getValue().head<2>(), 0).finished());
      trans.transform(pose, pose);
      pose(2) = dv_x[j]->getValue()(2);
      x_est_trans.push_back(pose);
      x_est_trans_log << pose.transpose() << std::endl;
    }

  return 0;
}
