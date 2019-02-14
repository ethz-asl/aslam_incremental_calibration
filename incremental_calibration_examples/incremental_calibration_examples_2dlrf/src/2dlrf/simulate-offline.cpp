/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file simulate-offline.cpp
    \brief This file runs a simulation of the calibration problem in batch mode.
  */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/three_point_methods.hpp>

#include <sm/BoostPropertyTree.hpp>

#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/statistics/UniformDistribution.h>
#include <aslam/calibration/statistics/NormalDistribution.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/geometry/Transformation.h>
#include <aslam/calibration/base/Timestamp.h>
#include <aslam-tsvd-solver/aslam-tsvd-solver.h>

#include "aslam/calibration/2dlrf/utils.h"
#include "aslam/calibration/2dlrf/ErrorTermMotion.h"
#include "aslam/calibration/2dlrf/ErrorTermObservation.h"

using namespace aslam::calibration;
using namespace aslam::backend;
using namespace sm::kinematics;
using namespace sm;

typedef aslam::backend::AslamTruncatedSvdSolver LinearSolver;
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <conf_file>" << std::endl;
    return -1;
  }

  // load configuration file
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[1]);

  // steps to simulate
  const size_t steps = propertyTree.getInt("lrf/problem/steps");

  // timestep size
  const double T = propertyTree.getDouble("lrf/problem/timestep");

  // true state
  std::vector<Eigen::Vector3d > x_true;
  x_true.reserve(steps);

  // integrated odometry
  std::vector<Eigen::Vector3d > x_odom;
  x_odom.reserve(steps);

  // true control input
  std::vector<Eigen::Vector3d > u_true;
  const double sineWaveAmplitude = propertyTree.getDouble(
    "lrf/problem/sineWaveAmplitude");
  const double sineWaveFrequency = propertyTree.getDouble(
    "lrf/problem/sineWaveFrequency");
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);

  // measured control input
  std::vector<Eigen::Vector3d > u_noise;
  u_noise.reserve(steps);

  // number of landmarks
  const size_t nl = propertyTree.getInt("lrf/problem/numLandmarks");

  // playground size
  const Eigen::Vector2d min(propertyTree.getDouble("lrf/problem/groundMinX"),
    propertyTree.getDouble("lrf/problem/groundMinY"));
  const Eigen::Vector2d max(propertyTree.getDouble("lrf/problem/groundMaxX"),
    propertyTree.getDouble("lrf/problem/groundMaxY"));

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
  Q(0, 0) = propertyTree.getDouble("lrf/problem/motion/sigma2_x");
  Q(1, 1) = propertyTree.getDouble("lrf/problem/motion/sigma2_y");
  Q(2, 2) = propertyTree.getDouble("lrf/problem/motion/sigma2_t");

  // covariance matrix for observation model
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = propertyTree.getDouble("lrf/problem/observation/sigma2_r");
  R(1, 1) = propertyTree.getDouble("lrf/problem/observation/sigma2_b");

  // landmark positions
  std::vector<Eigen::Vector2d > x_l;
  UniformDistribution<double, 2>(min, max).getSamples(x_l, nl);

  // true calibration parameters
  const Eigen::Vector3d Theta(propertyTree.getDouble("lrf/problem/thetaTrue/x"),
    propertyTree.getDouble("lrf/problem/thetaTrue/y"),
    propertyTree.getDouble("lrf/problem/thetaTrue/t"));

  // guessed calibration parameters
  const Eigen::Vector3d Theta_hat(
    propertyTree.getDouble("lrf/problem/thetaHat/x"),
    propertyTree.getDouble("lrf/problem/thetaHat/y"),
    propertyTree.getDouble("lrf/problem/thetaHat/t"));

  // initial state
  const Eigen::Vector3d x_0(propertyTree.getDouble("lrf/problem/x0/x"),
    propertyTree.getDouble("lrf/problem/x0/y"),
    propertyTree.getDouble("lrf/problem/x0/t"));
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
    xk(2) = angleMod(xk(2));
    x_true.push_back(xk);
    u_noise.push_back(u_true[i] +
      NormalDistribution<3>(Eigen::Vector3d::Zero(), Q).getSample());
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
  auto problem = boost::make_shared<OptimizationProblem>();

  // create calibration parameters design variable
  auto dv_Theta = boost::make_shared<VectorDesignVariable<3> >(Theta_hat);
  dv_Theta->setActive(true);
  problem->addDesignVariable(dv_Theta, 2);

  // create state design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<3> > > dv_x;
  dv_x.reserve(steps);
  for (size_t i = 0; i < steps; ++i) {
    dv_x.push_back(boost::make_shared<VectorDesignVariable<3> >(x_odom[i]));
    dv_x[i]->setActive(true);
    problem->addDesignVariable(dv_x[i], 0);
  }

  // create landmarks design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<2> > > dv_x_l;
  dv_x_l.reserve(nl);
  for (size_t i = 0; i < nl; ++i) {
    dv_x_l.push_back(boost::make_shared<VectorDesignVariable<2> >(x_l_hat[i]));
    dv_x_l[i]->setActive(true);
    problem->addDesignVariable(dv_x_l[i], 1);
  }

  // set the ordering of the problem
  problem->setGroupsOrdering({0, 1, 2});

  // add motion and observation error terms
  for (size_t i = 1; i < steps; ++i) {
    auto e_mot = boost::make_shared<ErrorTermMotion>(dv_x[i - 1].get(),
      dv_x[i].get(), T, u_noise[i], Q);
    problem->addErrorTerm(e_mot);
    for (size_t j = 0; j < nl; ++j) {
      auto e_obs = boost::make_shared<ErrorTermObservation>(dv_x[i].get(),
        dv_x_l[j].get(), dv_Theta.get(), r[i][j], b[i][j], R);
      problem->addErrorTerm(e_obs);
    }
  }

  // optimization
  std::cout << "Calibration before: " << *dv_Theta << std::endl;
  Optimizer2 optimizer(PropertyTree(propertyTree, "lrf/estimator/optimizer"),
    boost::make_shared<LinearSolver>(PropertyTree(propertyTree,
    "lrf/estimator/optimizer/linearSolver")),
    boost::make_shared<GaussNewtonTrustRegionPolicy>());
  optimizer.setProblem(problem);

  size_t JCols = 0;
  for (auto it = problem->getGroupsOrdering().cbegin();
      it != problem->getGroupsOrdering().cend(); ++it)
    JCols += problem->getGroupDim(*it);
  const size_t dim = problem->getGroupDim(2);
  auto linearSolver = optimizer.getSolver<LinearSolver>();
  linearSolver->setMargStartIndex(JCols - dim);
  const double before = Timestamp::now();
  optimizer.optimize();
  const double after = Timestamp::now();
  std::cout << "Elapsed time [s]: " << after - before << std::endl;
  std::cout << "Calibration after: " << *dv_Theta << std::endl;
  std::cout << "Singular values (scaled): "
    << linearSolver->getSingularValues().transpose() << std::endl;
  std::cout << "Unobservable basis (scaled): " << std::endl
    << linearSolver->getNullSpace() << std::endl;
  std::cout << "Observable basis (scaled): " << std::endl
    << linearSolver->getRowSpace() << std::endl;
  linearSolver->analyzeMarginal();
  std::cout << "SVD rank: " << linearSolver->getSVDRank() << std::endl;
  std::cout << "SVD rank deficiency: " << linearSolver->getSVDRankDeficiency()
    << std::endl;
  std::cout << "SVD tolerance: " << linearSolver->getSVDTolerance()
    << std::endl;
  std::cout << "Singular values: " << optimizer.getSolver<LinearSolver>()
    ->getSingularValues().transpose() << std::endl;
  std::cout << "QR rank: " << optimizer.getSolver<LinearSolver>()->getQRRank()
    << std::endl;
  std::cout << "QR rank deficiency: " << optimizer.getSolver<LinearSolver>()
    ->getQRRankDeficiency() << std::endl;
  std::cout << "QR tolerance: " << linearSolver->getQRTolerance() << std::endl;
  std::cout << "Unobservable basis: " << std::endl
    << linearSolver->getNullSpace() << std::endl;
  std::cout << "Observable basis: " << std::endl << linearSolver->getRowSpace()
    << std::endl;
  std::cout << "Covariance: " << std::endl << linearSolver->getCovariance()
    << std::endl;
  std::cout << "Observable covariance: " << std::endl
    << linearSolver->getRowSpaceCovariance() << std::endl;
  std::cout << "Peak memory usage (MB): " << linearSolver->getPeakMemoryUsage()
    / 1024.0 / 1024.0 << std::endl;
  std::cout << "Memory usage (MB): " << linearSolver->getMemoryUsage() /
    1024.0 / 1024.0 << std::endl;
  std::cout << "Flop count: " << linearSolver->getNumFlops() << std::endl;
  std::cout << "Linear solver time: " << linearSolver->getLinearSolverTime()
    << std::endl;
  std::cout << "Marginal analysis time: "
    << linearSolver->getMarginalAnalysisTime() << std::endl;
  std::cout << "Symbolic factorization time: "
    << linearSolver->getSymbolicFactorizationTime() << std::endl;
  std::cout << "Numeric factorization time: "
    << linearSolver->getNumericFactorizationTime() << std::endl;
  std::cout << "Log2sum of singular values: "
    << linearSolver->getSingularValuesLog2Sum() << std::endl;

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
  Transformation<double, 3> trans(threePointSvd(l, l_est));
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
