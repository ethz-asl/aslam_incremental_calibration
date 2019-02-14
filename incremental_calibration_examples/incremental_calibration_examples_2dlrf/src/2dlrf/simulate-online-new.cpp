/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file simulate-online-new.cpp
    \brief This file runs a simulation of the calibration problem in iterative
           mode.
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

#include <aslam/backend/CompressedColumnMatrix.hpp>

#include <aslam/calibration/statistics/UniformDistribution.h>
#include <aslam/calibration/statistics/NormalDistribution.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/geometry/Transformation.h>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/base/Timestamp.h>

#include "aslam/calibration/2dlrf/utils.h"
#include "aslam/calibration/2dlrf/ErrorTermMotion.h"
#include "aslam/calibration/2dlrf/ErrorTermObservation.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm;

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
  std::vector<Eigen::Vector3d> x_true;
  x_true.reserve(steps);

  // integrated odometry
  std::vector<Eigen::Vector3d> x_odom;
  x_odom.reserve(steps);

  // true control input
  std::vector<Eigen::Vector3d> u_true;
  const double sineWaveAmplitude = propertyTree.getDouble(
    "lrf/problem/sineWaveAmplitude");
  const double sineWaveFrequency = propertyTree.getDouble(
    "lrf/problem/sineWaveFrequency");
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);

  // measured control input
  std::vector<Eigen::Vector3d> u_noise;
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
  std::vector<Eigen::Vector2d> x_l;
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
  std::vector<Eigen::Vector2d> x_l_hat;
  initLandmarks(x_l_hat, x_odom, Theta_hat, r, b);

  // create landmarks design variables
  std::vector<boost::shared_ptr<VectorDesignVariable<2> > > dv_x_l;
  dv_x_l.reserve(nl);
  for (size_t i = 0; i < nl; ++i) {
    dv_x_l.push_back(boost::make_shared<VectorDesignVariable<2> >(x_l_hat[i]));
    dv_x_l[i]->setActive(true);
  }

  // create calibration parameters design variable
  auto dv_Theta = boost::make_shared<VectorDesignVariable<3> >(Theta_hat);
  dv_Theta->setActive(true);

  std::cout << "calibration before: " << *dv_Theta << std::endl;

  // create incremental estimator
  IncrementalEstimator incrementalEstimator(PropertyTree(propertyTree,
    "lrf/estimator"));

  // batch size
  const size_t batchSize = propertyTree.getInt("lrf/estimator/batchSize");

  // run over the dataset
  for (size_t i = 0; i < steps; i += batchSize) {

    // create batch
    auto batch = boost::make_shared<IncrementalEstimator::Batch>();

    // create starting state variable at k-1 and activate it
    auto dv_xkm1 = boost::make_shared<VectorDesignVariable<3> >(x_odom[i]);
    dv_xkm1->setActive(true);
    batch->addDesignVariable(dv_xkm1, 0);

    // add the calibration variable to the batch
    batch->addDesignVariable(dv_Theta, 2);

    // add the landmarks in the story
    for (size_t k = 0; k < nl; ++k)
      batch->addDesignVariable(dv_x_l[k], 1);

    // create other state variables and error terms
    for (size_t j = i + 1; j < i + batchSize && j < steps; ++j) {
      // create state variable at k and activate it
      auto dv_xk = boost::make_shared<VectorDesignVariable<3> >(x_odom[j]);
      dv_xk->setActive(true);

      // add the variables to the batch
      batch->addDesignVariable(dv_xk, 0);

      // motion error term
      auto e_mot = boost::make_shared<ErrorTermMotion>(dv_xkm1.get(),
        dv_xk.get(), T, u_noise[j], Q);
      batch->addErrorTerm(e_mot);

      // observation error terms
      for (size_t k = 0; k < nl; ++k) {
        auto e_obs = boost::make_shared<ErrorTermObservation>(dv_xk.get(),
          dv_x_l[k].get(), dv_Theta.get(), r[j][k], b[j][k], R);
        batch->addErrorTerm(e_obs);
      }
      // switch state variable
      dv_xkm1 = dv_xk;
    }

    // add the measurement batch to the estimator
    std::cout << "calibration before batch: " << *dv_Theta << std::endl;
    auto ret = incrementalEstimator.addBatch(batch);
    std::cout << "calibration after batch: " << *dv_Theta << std::endl;
    auto problem = incrementalEstimator.getProblem();
    std::cout << "ordering: ";
    for (auto it = problem->getGroupsOrdering().cbegin(); it !=
        problem->getGroupsOrdering().cend(); ++it)
      std::cout << *it << " ";
    std::cout << std::endl;
    ret.batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
    std::cout << std::endl;
    std::cout << "information gain: " << ret.informationGain << std::endl;
    std::cout << "rank of Psi: " << ret.rankPsi << std::endl;
    std::cout << "rank deficiency of Psi: " << ret.rankPsiDeficiency
      << std::endl;
    std::cout << "rank of Theta: " << ret.rankTheta << std::endl;
    std::cout << "rank deficiency of Theta: " << ret.rankThetaDeficiency
      << std::endl;
    std::cout << "unobservable basis: " << std::endl << ret.nobsBasis
      << std::endl;
    std::cout << "unobservable basis (scaled): " << std::endl
      << ret.nobsBasisScaled << std::endl;
    std::cout << "QR tolerance: " << ret.qrTolerance << std::endl;
    std::cout << "SVD tolerance: " << ret.svdTolerance << std::endl;
    std::cout << "time [s]: " << ret.elapsedTime << std::endl;
    std::cout << std::endl;
  }

  std::cout << "final calibration: " << *dv_Theta << std::endl;
  std::cout << "covariance: " << std::endl <<
    incrementalEstimator.getSigma2Theta() << std::endl;

  // for debugging purpose, write the Jacobian to file
  std::ofstream jacobianFile("J.txt");
  incrementalEstimator.getJacobianTranspose().writeMATLAB(jacobianFile);

  // for debugging purpose, output some infos
  std::cout << "information gain: " << incrementalEstimator.getInformationGain()
    << std::endl;
  std::cout << "rank of Psi: " << incrementalEstimator.getRankPsi()
    << std::endl;
  std::cout << "rank deficiency of Psi: "
    << incrementalEstimator.getRankPsiDeficiency() << std::endl;
  std::cout << "rank of Theta: " << incrementalEstimator.getRankTheta()
    << std::endl;
  std::cout << "rank deficiency of Theta: "
    << incrementalEstimator.getRankThetaDeficiency() << std::endl;
  std::cout << "QR tolerance: " << incrementalEstimator.getQRTolerance()
    << std::endl;
  std::cout << "SVD tolerance: " << incrementalEstimator.getSVDTolerance()
    << std::endl;
  std::cout << "unobservable basis: " << std::endl
    << incrementalEstimator.getNobsBasis() << std::endl;
    std::cout << "unobservable basis (scaled): " << std::endl
    << incrementalEstimator.getNobsBasis(true) << std::endl;
  std::cout << "observable basis: " << std::endl
    << incrementalEstimator.getObsBasis() << std::endl;
  std::cout << "observable covariance: " << std::endl
    << incrementalEstimator.getSigma2ThetaObs() << std::endl;
  std::cout << "memory usage [MB]: " << incrementalEstimator.getMemoryUsage() /
    1024.0 / 1024.0 << std::endl;
  std::cout << "peak memory usage [MB]: "
    << incrementalEstimator.getPeakMemoryUsage() / 1024.0 / 1024.0 << std::endl;
  std::cout << "marginalized group: " << incrementalEstimator.getMargGroupId()
    << std::endl;
  std::cout << "number of batches: " << incrementalEstimator.getNumBatches()
    << " out of " << round((double)steps / batchSize) << std::endl;

  // fetch the problem
  auto problem = incrementalEstimator.getProblem();

  // some other output
  std::cout << "number of design variables: " << problem->numDesignVariables()
    << std::endl;
  std::cout << "number of error terms: " << problem->numErrorTerms()
    << std::endl;
  std::cout << "Jacobian matrix is: "
    << incrementalEstimator.getJacobianTranspose().cols() << "x"
    << incrementalEstimator.getJacobianTranspose().rows() << std::endl;

  // output results to file
  std::ofstream x_true_log("x_true.txt");
  for (size_t i = 0; i < steps; ++i)
    x_true_log << x_true[i].transpose() << std::endl;
  std::ofstream x_odom_log("x_odom.txt");
  for (size_t i = 0; i < steps; ++i)
    x_odom_log << x_odom[i].transpose() << std::endl;
  std::ofstream l_log("l.txt");
  for (size_t i = 0; i < nl; ++i)
    l_log << x_l[i].transpose() << std::endl;

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
  std::vector<Eigen::Vector3d> x_est_trans;
  x_est_trans.reserve(steps);
  std::ofstream x_est_trans_log("x_est_trans.txt");
  const IncrementalOptimizationProblem::DesignVariablesP&
    dvsx = problem->getDesignVariablesGroup(0);
  for (auto it = dvsx.cbegin(); it != dvsx.cend(); ++ it) {
    const VectorDesignVariable<3>* dv_xk =
      dynamic_cast<const VectorDesignVariable<3>*>(*it);
    Eigen::Vector3d pose((Eigen::Vector3d()
      << dv_xk->getValue().head<2>(), 0).finished());
    trans.transform(pose, pose);
    pose(2) = dv_xk->getValue()(2);
    x_est_trans.push_back(pose);
    x_est_trans_log << pose.transpose() << std::endl;
  }

  return 0;
}
