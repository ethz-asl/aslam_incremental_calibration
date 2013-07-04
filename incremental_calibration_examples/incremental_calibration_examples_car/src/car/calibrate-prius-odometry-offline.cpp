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

/** \file calibrate-prius-odometry-offline.cpp
    \brief This file calibrates the odometry parameters of a Toyota PRIUS
           offline.
  */

#include <iostream>
#include <fstream>
#include <iomanip>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>

#include <bsplines/BSplinePose.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/CANBinaryParser.h"
#include "aslam/calibration/car/ApplanixBinaryParser.h"
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/car/ErrorTermFws.h"
#include "aslam/calibration/car/ErrorTermRws.h"
#include "aslam/calibration/car/ErrorTermSteering.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace bsplines;
using namespace aslam::splines;
using namespace aslam::backend;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
      << " <applanix-binary-log> <can-binary-log>" << std::endl;
    return -1;
  }

  // parse the Applanix log file
  std::cout << "Parsing Applanix data..." << std::endl;
  ApplanixBinaryParser applanixParser(argv[1]);
  applanixParser.parse();

  // parse the CAN log file
  std::cout << "Parsing CAN data..." << std::endl;
  CANBinaryParser canParser(argv[2]);
  canParser.parse();

  // create rotation parameterizations
  boost::shared_ptr<RotationVector> rv(new RotationVector);
  const EulerAnglesYawPitchRoll ypr;

  // create B-Spline
  const int order = 4;
  BSplinePose bspline(order, rv);

  // iterate over the parsed Applanix messages
  std::cout << "Creating data for B-spline..." << std::endl;
  const size_t numMeasurements = applanixParser.getNumNavigationSolution();
  Eigen::VectorXd timestamps(numMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
  size_t i = 0;
  for (auto it = applanixParser.cbegin(); it != applanixParser.cend(); ++it) {
    Eigen::Vector3d crv = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(it->second.yaw,
      it->second.pitch, it->second.roll)));
    // ensure rotation vector do not flip
    if (i > 0) {
      Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
      crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
    }
    timestamps(i) = it->first;
    Eigen::Matrix<double, 6, 1> pose;
    pose << it->second.x, it->second.y, it->second.z, crv;
    poses.col(i) = pose;
    ++i;
  }

  std::cout << "Number of measurements: " << numMeasurements << std::endl;
  const double elapsedTime =
    timestamps(numMeasurements - 1) - timestamps(0);
  std::cout << "Sequence length [s]: " << elapsedTime << std::endl;

  // fill the B-Spline with measurements
  const double lambda = 1e-1;
  const int measPerSec = numMeasurements / elapsedTime;
  const int measPerSecDesired = 5;
  int numSegments;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = numMeasurements;
  std::cout << "Creating B-Spline with " << numSegments << " segments..."
    << std::endl;
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);

  // create optimization problem
  boost::shared_ptr<aslam::backend::OptimizationProblem> problem(
    new aslam::backend::OptimizationProblem);

  // create B-spline design variables
  std::cout << "Creating B-spline design variables..." << std::endl;
  boost::shared_ptr<BSplinePoseDesignVariable> bspdv(
    new BSplinePoseDesignVariable(bspline));
  for (size_t i = 0; i < bspdv->numDesignVariables(); ++i) {
    bspdv->designVariable(i)->setActive(true);
    problem->addDesignVariable(bspdv->designVariable(i), false);
  }

  // create error terms for each Applanix measurement
  std::cout << "Creating error terms for Applanix measurements..." << std::endl;
  i = 0;
  for (auto it = applanixParser.cbegin(); it != applanixParser.cend(); ++it) {
    ErrorTermPose::Input xm;
    xm.head<3>() = Eigen::Vector3d(it->second.x, it->second.y, it->second.z);
    xm.tail<3>() = Eigen::Vector3d(it->second.yaw, it->second.pitch,
      it->second.roll);
    aslam::calibration::ErrorTermPose::Covariance Q =
      aslam::calibration::ErrorTermPose::Covariance::Zero();
    Q(0, 0) = it->second.x_sigma2;
    Q(1, 1) = it->second.y_sigma2;
    Q(2, 2) = it->second.z_sigma2;
    Q(3, 3) = it->second.yaw_sigma2;
    Q(4, 4) = it->second.pitch_sigma2;
    Q(5, 5) = it->second.roll_sigma2;
    boost::shared_ptr<ErrorTermPose> e_pose(new ErrorTermPose(
      bspdv->transformation(timestamps(i)), xm, Q));
    problem->addErrorTerm(e_pose);
    ++i;
  }

  // transformation between IMU and odometry design variable
  boost::shared_ptr<EuclideanPoint> t_io_dv(
    new EuclideanPoint(Eigen::Vector3d(0, 0, -0.785)));
  t_io_dv->setActive(true);
  boost::shared_ptr<RotationQuaternion> C_io_dv(
    new RotationQuaternion(
    ypr.parametersToRotationMatrix(Eigen::Vector3d(0, 0, 0))));
  C_io_dv->setActive(true);
  RotationExpression C_io(C_io_dv);
  EuclideanExpression t_io(t_io_dv);
  problem->addDesignVariable(t_io_dv);
  problem->addDesignVariable(C_io_dv);

  // intrisic odometry parameters
  const double L = 2.7; // wheelbase [m]
  const double e_r = 0.7575; // half-track rear [m]
  const double e_f = 0.7625; // half-track front [m]
  const double a0 = 0; // steering coefficient
  const double a1 = 1.0 / (M_PI / 180 / 10); // steering coefficient
  const double a2 = 0; // steering coefficient
  const double a3 = 0; // steering coefficient
  const double k_rl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_rr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fr = 1.0 / 3.6 / 100.0; // wheel coefficient
  boost::shared_ptr<VectorDesignVariable<11> > cpdv(
    new VectorDesignVariable<11>(
    (VectorDesignVariable<11>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr).finished()));
  cpdv->setActive(true);
  problem->addDesignVariable(cpdv);

  std::cout << "Creating error terms for rear wheel speed measurements..."
    << std::endl;
  Eigen::Matrix2d R_rws;
  R_rws << 828.2516524610561, 814.5506263601094, 814.5506263601094,
    990.2353478304882;
  std::ofstream errorsRws("errors-rws.txt");
  std::ofstream errorsRwsRaw("errors-rws-raw.txt");
  for (auto it = canParser.cbeginRw(); it != canParser.cendRw(); ++it) {
    if (it->second.first == 0 || it->second.second == 0)
      continue;
    EuclideanExpression v_iw = bspdv->linearVelocity(it->first);
    EuclideanExpression om_ii = bspdv->angularVelocityBodyFrame(it->first);
    RotationExpression C_wi = bspdv->orientation(it->first);
    EuclideanExpression v_ii = C_wi.inverse() * v_iw;
    EuclideanExpression v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    EuclideanExpression om_oo = C_io.inverse() * om_ii;
    boost::shared_ptr<ErrorTermRws> e_rws(new ErrorTermRws(v_oo, om_oo,
      cpdv.get(), Eigen::Vector2d(it->second.second, it->second.first), R_rws));
    problem->addErrorTerm(e_rws);
//    e_rws->setMEstimatorPolicy(
//      boost::shared_ptr<BlakeZissermanMEstimator>(
//      new BlakeZissermanMEstimator(e_rws->dimension(), 0.999, 0.1)));
    errorsRws << e_rws->evaluateError() << std::endl;
    errorsRwsRaw << e_rws->error().transpose() << std::endl;
  }

  std::cout << "Creating error terms for front wheel speed measurements..."
    << std::endl;
  Eigen::Matrix2d R_fws;
  R_fws << 1225.545759661739, 1185.490847789361, 1185.490847789361,
    1271.17379804095;
  std::ofstream errorsFws("errors-fws.txt");
  std::ofstream errorsFwsRaw("errors-fws-raw.txt");
  for (auto it = canParser.cbeginFw(); it != canParser.cendFw(); ++it) {
    if (it->second.first == 0 || it->second.second == 0)
      continue;
    EuclideanExpression v_iw = bspdv->linearVelocity(it->first);
    EuclideanExpression om_ii = bspdv->angularVelocityBodyFrame(it->first);
    RotationExpression C_wi = bspdv->orientation(it->first);
    EuclideanExpression v_ii = C_wi.inverse() * v_iw;
    EuclideanExpression v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    EuclideanExpression om_oo = C_io.inverse() * om_ii;
    boost::shared_ptr<ErrorTermFws> e_fws(new ErrorTermFws(v_oo, om_oo,
      cpdv.get(), Eigen::Vector2d(it->second.second, it->second.first), R_fws));
    problem->addErrorTerm(e_fws);
//    e_fws->setMEstimatorPolicy(
//      boost::shared_ptr<BlakeZissermanMEstimator>(
//      new BlakeZissermanMEstimator(e_fws->dimension(), 0.999, 0.1)));
    errorsFws << e_fws->evaluateError() << std::endl;
    errorsFwsRaw << e_fws->error().transpose() << std::endl;
  }

  std::cout << "Creating error terms for steering measurements..."
    << std::endl;
  Eigen::Matrix<double, 1, 1> R_st;
  R_st << 3.369624218217728;
  std::ofstream errorsSt("errors-st.txt");
  std::ofstream errorsStRaw("errors-st-raw.txt");
  for (auto it = canParser.cbeginSt1(); it != canParser.cendSt1(); ++it) {
    EuclideanExpression v_iw = bspdv->linearVelocity(it->first);
    EuclideanExpression om_ii = bspdv->angularVelocityBodyFrame(it->first);
    RotationExpression C_wi = bspdv->orientation(it->first);
    EuclideanExpression v_ii = C_wi.inverse() * v_iw;
    EuclideanExpression v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    EuclideanExpression om_oo = C_io.inverse() * om_ii;
    Eigen::Matrix<double, 1, 1> meas;
    meas << it->second;
    boost::shared_ptr<ErrorTermSteering> e_st(new ErrorTermSteering(v_oo, om_oo,
      cpdv.get(), meas, R_st));
    problem->addErrorTerm(e_st);
    errorsSt << e_st->evaluateError() << std::endl;
    errorsStRaw << e_st->error().transpose() << std::endl;
  }

  // optimize
  std::cout << "Optimizing..." << std::endl;
  std::cout << "Initial guess: " << std::endl;
  std::cout << "Odometry parameters: " << std::endl;
  std::cout << std::fixed << std::setprecision(16) << *cpdv << std::endl;
  std::cout << "Translation IMU-ODO: " << std::endl;
  std::cout << t_io.toValue().transpose() << std::endl;
  std::cout << "Rotation IMU-ODO: " << std::endl;
  std::cout <<
    ypr.rotationMatrixToParameters(C_io.toRotationMatrix()).transpose()
    << std::endl;
  aslam::backend::Optimizer2Options options;
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
  std::cout << "Estimated parameters: " << std::endl;
  std::cout << "Odometry parameters: " << std::endl;
  std::cout << *cpdv << std::endl;
  std::cout << "Translation IMU-ODO: " << std::endl;
  std::cout << t_io.toValue().transpose() << std::endl;
  std::cout << "Rotation IMU-ODO: " << std::endl;
  std::cout <<
    ypr.rotationMatrixToParameters(C_io.toRotationMatrix()).transpose()
    << std::endl;

  return 0;
}
