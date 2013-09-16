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

/** \file calibrateCarOdometrySimulationBatch.cpp
    \brief This file calibrates the car parameters from simulated data.
  */

#include <iostream>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/transformations.hpp>

#include <sm/timing/TimestampCorrector.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>

#include <libposlv/geo-tools/Geo.h>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/algorithms/matrixOperations.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/ApplanixDMIMeasurement.h"
#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/car/ErrorTermFws.h"
#include "aslam/calibration/car/ErrorTermRws.h"
#include "aslam/calibration/car/ErrorTermSteering.h"
#include "aslam/calibration/car/ErrorTermDMI.h"

struct WheelsSpeedMeasurement {
  double left;
  double right;
};
struct SteeringMeasurement {
  double value;
};

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace aslam::backend;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file>" << std::endl;
    return -1;
  }

  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  topics.push_back(std::string("/poslv/vehicle_navigation_solution"));
  topics.push_back(std::string("/poslv/vehicle_navigation_performance"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::cout << "Processing BAG file..." << std::endl;
  poslv::VehicleNavigationPerformanceMsgConstPtr lastVnp;
  bool firstVNS = true;
  double latRef = 0;
  double longRef = 0;
  double altRef = 0;
  size_t viewCounter = 0;
  MeasurementsContainer<ApplanixNavigationMeasurement>::Type
    navigationMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    frontWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    rearWheelsSpeedMeasurements;
  MeasurementsContainer<SteeringMeasurement>::Type steeringMeasurements;
  MeasurementsContainer<ApplanixDMIMeasurement>::Type encoderMeasurements;
  TimestampCorrector<double> timestampCorrector;
  std::vector<boost::shared_ptr<EuclideanPoint> > linearVelocities;
  std::vector<boost::shared_ptr<EuclideanPoint> > angularVelocities;
  std::vector<boost::shared_ptr<EuclideanPoint> > translations;
  std::vector<boost::shared_ptr<RotationQuaternion> > rotations;
  auto ypr = boost::make_shared<EulerAnglesYawPitchRoll>();
  const double e_r = 0.74;
  const double k_rl = 1.0 / 3.6 / 100.0;
  const double k_rr = 1.0 / 3.6 / 100.0;
  const double e_f = 0.755;
  const double k_fl = 1.0 / 3.6 / 100.0;
  const double k_fr = 1.0 / 3.6 / 100.0;
  const double L = 2.7;
  const double a0 = 0;
  const double a1 = (M_PI / 180 / 10);
  const double a2 = 0;
  const double a3 = 0;
  const double sigma2_rl = 1;
  const double sigma2_rr = 1;
  const double sigma2_fl = 1;
  const double sigma2_fr = 1;
  const double sigma2_st = 1;
  const double sigma2_dmi = 1;
  const Eigen::Vector3d t_io_t(0.0, 0.0, -0.785);
  const Eigen::Matrix3d C_io_t(ypr->parametersToRotationMatrix(
    Eigen::Vector3d(deg2rad(0), deg2rad(0), deg2rad(0))));
  const Eigen::Matrix4d T_io_t(Transformation(r2quat(C_io_t), t_io_t).T());
  auto problem = boost::make_shared<OptimizationProblem>();
  double traveledDistance = 0;
  Eigen::Matrix4d T_wi_km1_t;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->isType<poslv::VehicleNavigationPerformanceMsg>()) {
      poslv::VehicleNavigationPerformanceMsgConstPtr vnp(
        it->instantiate<poslv::VehicleNavigationPerformanceMsg>());
      lastVnp = vnp;
    }
    if (it->isType<poslv::VehicleNavigationSolutionMsg>()) {
      if (!lastVnp)
        continue;
      poslv::VehicleNavigationSolutionMsgConstPtr vns(
        it->instantiate<poslv::VehicleNavigationSolutionMsg>());
      if (firstVNS) {
        latRef = vns->latitude;
        longRef = vns->longitude;
        altRef = vns->altitude;
        firstVNS = false;
      }
      double x_ecef, y_ecef, z_ecef;
      Geo::wgs84ToEcef(vns->latitude, vns->longitude, vns->altitude, x_ecef,
        y_ecef, z_ecef);
      double x_enu, y_enu, z_enu;
      Geo::ecefToEnu(x_ecef, y_ecef, z_ecef, latRef, longRef, altRef, x_enu,
        y_enu, z_enu);
      ApplanixNavigationMeasurement data;
      data.x = x_enu;
      data.y = y_enu;
      data.z = z_enu;
      data.yaw = angleMod(deg2rad(-vns->heading) + M_PI / 2);
      data.pitch = deg2rad(-vns->pitch);
      data.roll = deg2rad(vns->roll);
      Eigen::Vector3d linearVelocity =
        Geo::R_ENU_NED::getInstance().getMatrix() * Eigen::Vector3d(
        vns->northVelocity, vns->eastVelocity, vns->downVelocity);
      data.v_x = linearVelocity(0);
      data.v_y = linearVelocity(1);
      data.v_z = linearVelocity(2);
      data.om_x = deg2rad(vns->angularRateLong);
      data.om_y = -deg2rad(vns->angularRateTrans);
      data.om_z = -deg2rad(vns->angularRateDown);
      data.a_x = vns->accLong;
      data.a_y = -vns->accTrans;
      data.a_z = -vns->accDown;
      data.v = vns->speed;
      data.x_sigma2 = lastVnp->eastPositionRMSError *
        lastVnp->eastPositionRMSError;
      data.y_sigma2 = lastVnp->northPositionRMSError *
        lastVnp->northPositionRMSError;
      data.z_sigma2 = lastVnp->downPositionRMSError *
        lastVnp->downPositionRMSError;
      data.roll_sigma2 = deg2rad(lastVnp->rollRMSError) *
        deg2rad(lastVnp->rollRMSError);
      data.pitch_sigma2 = deg2rad(lastVnp->pitchRMSError) *
        deg2rad(lastVnp->pitchRMSError);
      data.yaw_sigma2 = deg2rad(lastVnp->headingRMSError) *
        deg2rad(lastVnp->headingRMSError);
      data.v_x_sigma2 = lastVnp->eastVelocityRMSError *
        lastVnp->eastVelocityRMSError;
      data.v_y_sigma2 = lastVnp->northVelocityRMSError *
        lastVnp->northVelocityRMSError;
      data.v_z_sigma2 = lastVnp->downVelocityRMSError *
        lastVnp->downVelocityRMSError;
      navigationMeasurements.push_back(
        std::make_pair(round(timestampCorrector.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())),
        data));
      const Eigen::Vector3d v_iw(data.v_x, data.v_y, data.v_z);
      const Eigen::Matrix3d C_wi = ypr->parametersToRotationMatrix(
        Eigen::Vector3d(data.yaw, data.pitch, data.roll));
      const Eigen::Vector3d v_ii = C_wi.transpose() * v_iw;
      linearVelocities.push_back(boost::make_shared<EuclideanPoint>(v_ii));
      problem->addDesignVariable(linearVelocities.back());
      const Eigen::Vector3d om_ii(data.om_x, data.om_y, data.om_z);
      angularVelocities.push_back(boost::make_shared<EuclideanPoint>(om_ii));
      problem->addDesignVariable(angularVelocities.back());
      const Eigen::Vector3d t_wi(data.x, data.y, data.z);
      translations.push_back(boost::make_shared<EuclideanPoint>(t_wi));
      problem->addDesignVariable(translations.back());
      rotations.push_back(boost::make_shared<RotationQuaternion>(C_wi));
      problem->addDesignVariable(rotations.back());
      const Eigen::Vector3d v_oo = C_io_t.transpose() *
        (v_ii + om_ii.cross(t_io_t));
      const Eigen::Vector3d om_oo = C_io_t.transpose() * om_ii;
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
      const double fl = (v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl;
      const double fr = (v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr;
      const double rl = (v_oo_x - e_r * om_oo_z) / k_rl;
      const double rr = (v_oo_x + e_r * om_oo_z) / k_rr;
      const double phi = atan(L * om_oo_z / v_oo_x);
      const double st = (phi - a0) / a1;
//      WheelsSpeedMeasurement fws = {fl + NormalDistribution<1>(0, sigma2_fl).getSample(), fr + NormalDistribution<1>(0, sigma2_fr).getSample()};
      WheelsSpeedMeasurement fws = {fl, fr};
      frontWheelsSpeedMeasurements.push_back(
        std::make_pair(navigationMeasurements.back().first, fws));
//      WheelsSpeedMeasurement rws = {rl + NormalDistribution<1>(0, sigma2_rl).getSample(), rr + NormalDistribution<1>(0, sigma2_rr).getSample()};
      WheelsSpeedMeasurement rws = {rl, rr};
      rearWheelsSpeedMeasurements.push_back(
        std::make_pair(navigationMeasurements.back().first, rws));
//      SteeringMeasurement steering = {st + NormalDistribution<1>(0, sigma2_st).getSample()};
      SteeringMeasurement steering = {st};
      steeringMeasurements.push_back(
        std::make_pair(navigationMeasurements.back().first, steering));
      const Eigen::Matrix4d T_wi_k(Transformation(r2quat(C_wi),
        Eigen::Vector3d(data.x, data.y, data.z)).T());
      if (navigationMeasurements.size() > 1) {
        const Eigen::Matrix4d T_o_km1_o_k = T_io_t.inverse() *
          T_wi_km1_t.inverse() * T_wi_k * T_io_t;
        const Eigen::Vector3d t_o_km1_o_k = transform2rho(T_o_km1_o_k);
        const Eigen::Matrix3d C_o_km1_o_k = transform2C(T_o_km1_o_k);
        const double delta = t_o_km1_o_k(0);
        const double omega = (ypr->rotationMatrixToParameters(C_o_km1_o_k))(0);
        const double displacement = (delta - e_r * omega);
        traveledDistance += displacement;
        ApplanixDMIMeasurement dmi = {traveledDistance, 0};
        encoderMeasurements.push_back(
          std::make_pair(navigationMeasurements.back().first, dmi));
      }
      T_wi_km1_t = T_wi_k;
    }
  }

  std::cout << "Building optimization problem..." << std::endl;
  auto cpdv = boost::make_shared<VectorDesignVariable<11> >(
    (VectorDesignVariable<11>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr).finished());
  cpdv->setActive(true);
  auto t_io_dv = boost::make_shared<EuclideanPoint>(t_io_t);
  t_io_dv->setActive(true);
  auto C_io_dv = boost::make_shared<RotationQuaternion>(C_io_t);
  C_io_dv->setActive(true);
  RotationExpression C_io(C_io_dv);
  EuclideanExpression t_io(t_io_dv);
  problem->addDesignVariable(t_io_dv);
  problem->addDesignVariable(C_io_dv);
  problem->addDesignVariable(cpdv);
  auto T_io = TransformationExpression(C_io, t_io);
  TransformationExpression T_wi_km1;
  double lastDistance = -1;
  for (size_t i = 0; i < linearVelocities.size(); ++i) {
    auto v_ii = EuclideanExpression(linearVelocities[i]);
    auto om_ii = EuclideanExpression(angularVelocities[i]);
    auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    auto om_oo = C_io.inverse() * om_ii;
    const double v_oo_x = v_oo.toValue()(0);
    const double om_oo_z = om_oo.toValue()(2);
    const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
    const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
    if (fabs(cos(phi_L)) > 1e-6 && fabs(cos(phi_R)) > 1e-6) {
      auto e_fws = boost::make_shared<ErrorTermFws>(v_oo, om_oo, cpdv.get(),
        Eigen::Vector2d(frontWheelsSpeedMeasurements[i].second.left,
        frontWheelsSpeedMeasurements[i].second.right),
        (Eigen::Matrix2d() << sigma2_fl, 0, 0, sigma2_fr).finished());
      problem->addErrorTerm(e_fws);
    }
    auto e_rws = boost::make_shared<ErrorTermRws>(v_oo, om_oo, cpdv.get(),
      Eigen::Vector2d(rearWheelsSpeedMeasurements[i].second.left,
      rearWheelsSpeedMeasurements[i].second.right),
      (Eigen::Matrix2d() << sigma2_rl, 0, 0, sigma2_rr).finished());
    problem->addErrorTerm(e_rws);
    if (std::fabs(v_oo_x) > 1e-1) {
      Eigen::Matrix<double, 1, 1> meas;
      meas << steeringMeasurements[i].second.value;
      auto e_st = boost::make_shared<ErrorTermSteering>(v_oo, om_oo, cpdv.get(),
        meas, (Eigen::Matrix<double, 1, 1>() << sigma2_st).finished());
      problem->addErrorTerm(e_st);
    }
    auto t_wi = EuclideanExpression(translations[i]);
    auto C_wi = RotationExpression(rotations[i]);
    auto T_wi_k = TransformationExpression(C_wi, t_wi);
    if (i > 0) {
      const double displacement =
        encoderMeasurements[i - 1].second.signedDistanceTraveled - lastDistance;
      const Eigen::Matrix<double, 1, 1> meas((Eigen::Matrix<double, 1, 1>()
        << displacement).finished());
      auto T_o_km1_o_k = T_io.inverse() * T_wi_km1.inverse() * T_wi_k * T_io;
      auto t_o_km1_o_k = T_o_km1_o_k.toEuclideanExpression();
      auto C_o_km1_o_k = T_o_km1_o_k.toRotationExpression();
      auto ypr_o_km1_o_k = C_o_km1_o_k.toParameters(ypr);
      auto e_dmi = boost::make_shared<ErrorTermDMI>(t_o_km1_o_k, ypr_o_km1_o_k,
        cpdv.get(), meas,
        (Eigen::Matrix<double, 1, 1>() << sigma2_dmi).finished());
//      problem->addErrorTerm(e_dmi);
    }
    T_wi_km1 = T_wi_k;
    lastDistance = encoderMeasurements[i - 1].second.signedDistanceTraveled;
  }

  std::cout << "Calibration before optimization: " << std::endl;
  std::cout << "CAN intrinsic: " << std::fixed << std::setprecision(18)
    << *cpdv << std::endl;
  std::cout << "Translation IMU-ODO: " << std::endl;
  std::cout << std::fixed << std::setprecision(18)
    << t_io.toValue().transpose() << std::endl;
  std::cout << "Rotation IMU-ODO: " << std::endl;
  std::cout << std::fixed << std::setprecision(18)
    << ypr->rotationMatrixToParameters(C_io.toRotationMatrix()).transpose()
    << std::endl;

  std::cout << "Optimizing..." << std::endl;
  Optimizer2Options options;
  options.verbose = true;
  options.linearSystemSolver = boost::make_shared<SparseQrLinearSystemSolver>();
  options.trustRegionPolicy =
    boost::make_shared<GaussNewtonTrustRegionPolicy>();
  SparseQRLinearSolverOptions linearSolverOptions;
  linearSolverOptions.colNorm = true;
  linearSolverOptions.qrTol = 0.02;
  Optimizer2 optimizer(options);
  optimizer.getSolver<SparseQrLinearSystemSolver>()->setOptions(
    linearSolverOptions);
  optimizer.setProblem(problem);
  optimizer.initialize();
  optimizer.getSolver<SparseQrLinearSystemSolver>()->buildSystem(
    optimizer.options().nThreads, true);
  const CompressedColumnMatrix<ssize_t>& JInit =
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getJacobianTranspose();
  std::ofstream JInitFile("JInit.txt");
  JInit.writeMATLAB(JInitFile);
  optimizer.optimize();

  std::cout << "Calibration after optimization: " << std::endl;
  std::cout << "CAN intrinsic: " << std::fixed << std::setprecision(18)
    << *cpdv << std::endl;
  std::cout << "Translation IMU-ODO: " << std::endl;
  std::cout << std::fixed << std::setprecision(18)
    << t_io.toValue().transpose() << std::endl;
  std::cout << "Rotation IMU-ODO: " << std::endl;
  std::cout << std::fixed << std::setprecision(18)
    << ypr->rotationMatrixToParameters(C_io.toRotationMatrix()).transpose()
    << std::endl;
  const CompressedColumnMatrix<ssize_t>& RFactor =
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getR();
  const size_t numCols = RFactor.cols();
  std::cout << "Sigma: " << std::endl
    << computeCovariance(RFactor, 0, numCols - 1).
    diagonal().transpose() << std::endl;
  std::ofstream RFile("R.txt");
  RFactor.writeMATLAB(RFile);
  const CompressedColumnMatrix<ssize_t>& JOpt =
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getJacobianTranspose();
  std::ofstream JOptFile("JOpt.txt");
  JOpt.writeMATLAB(JOptFile);
  std::cout << "Rank: " << optimizer.getSolver<SparseQrLinearSystemSolver>()
    ->getRank() << std::endl;

  return 0;
}
