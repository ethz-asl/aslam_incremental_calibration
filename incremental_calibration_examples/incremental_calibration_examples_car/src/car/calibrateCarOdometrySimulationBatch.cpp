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
#include <limits>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

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
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/BSplineFitter.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>

#include <libposlv/geo-tools/Geo.h>

#include <robot-odometry/generic/DifferentialOdometry.h>

#include <aslam/DiscreteTrajectory.hpp>
#include <aslam/SplineTrajectory.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/algorithms/marginalize.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/SteeringMeasurement.h"
#include "aslam/calibration/car/ApplanixDMIMeasurement.h"
#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/car/ErrorTermFws.h"
#include "aslam/calibration/car/ErrorTermRws.h"
#include "aslam/calibration/car/ErrorTermSteering.h"
#include "aslam/calibration/car/ErrorTermDMI.h"
#include "aslam/calibration/car/ErrorTermVehicleModel.h"
#include "aslam/calibration/car/utils.h"

using namespace aslam;
using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace bsplines;
using namespace aslam::splines;
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
  TimestampCorrector<double> timestampCorrector;
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
      double x_ned, y_ned, z_ned;
      Geo::ecefToNed(x_ecef, y_ecef, z_ecef, latRef, longRef, altRef, x_ned,
        y_ned, z_ned);
      ApplanixNavigationMeasurement data;
      data.x = x_ned;
      data.y = y_ned;
      data.z = z_ned;
      data.yaw = angleMod(deg2rad(vns->heading));
      data.pitch = deg2rad(vns->pitch);
      data.roll = deg2rad(vns->roll);
      data.v_x = vns->northVelocity;
      data.v_y = vns->eastVelocity;
      data.v_z = vns->downVelocity;
      data.om_x = deg2rad(vns->angularRateLong);
      data.om_y = deg2rad(vns->angularRateTrans);
      data.om_z = deg2rad(vns->angularRateDown);
      data.a_x = vns->accLong;
      data.a_y = vns->accTrans;
      data.a_z = vns->accDown;
      data.v = vns->speed;
      data.x_sigma2 = lastVnp->northPositionRMSError *
        lastVnp->northPositionRMSError;
      data.y_sigma2 = lastVnp->eastPositionRMSError *
        lastVnp->eastPositionRMSError;
      data.z_sigma2 = lastVnp->downPositionRMSError *
        lastVnp->downPositionRMSError;
      data.roll_sigma2 = deg2rad(lastVnp->rollRMSError) *
        deg2rad(lastVnp->rollRMSError);
      data.pitch_sigma2 = deg2rad(lastVnp->pitchRMSError) *
        deg2rad(lastVnp->pitchRMSError);
      data.yaw_sigma2 = deg2rad(lastVnp->headingRMSError) *
        deg2rad(lastVnp->headingRMSError);
      data.v_x_sigma2 = lastVnp->northVelocityRMSError *
        lastVnp->northVelocityRMSError;
      data.v_y_sigma2 = lastVnp->eastVelocityRMSError *
        lastVnp->eastVelocityRMSError;
      data.v_z_sigma2 = lastVnp->downVelocityRMSError *
        lastVnp->downVelocityRMSError;
      navigationMeasurements.push_back(
        std::make_pair(round(timestampCorrector.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())),
        data));
    }
  }

  std::cout << "Simulating data..." << std::endl;
  auto ypr = boost::make_shared<EulerAnglesYawPitchRoll>();
  DiscreteTrajectory discreteTrajectory;
  generateTrajectory(navigationMeasurements, discreteTrajectory);
  const double lambda = 0;
  const int knotsPerSecond = 5;
  const int splineOrder = 4;
  SplineTrajectory splineTrajectory(splineOrder);
  splineTrajectory.initialize(discreteTrajectory, knotsPerSecond, lambda);
  const double freqW = 60;
  const double sigma2_rl = 100;
  const double sigma2_rr = 100;
  const double e_r = 0.74;
  const double k_rl = 1.0 / 3.6 / 100.0;
  const double k_rr = 1.0 / 3.6 / 100.0;
  const Transformation T_io(r2quat(ypr->parametersToRotationMatrix(
    Eigen::Vector3d(0, 0, M_PI))),
    Eigen::Vector3d(0, 0, 0.785));
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueRearWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    rearWheelsSpeedMeasurements;
  simulateRearWheelsSpeedMeasurements(splineTrajectory, freqW, sigma2_rl,
    sigma2_rr, e_r, k_rl, k_rr, T_io, trueRearWheelsSpeedMeasurements,
    rearWheelsSpeedMeasurements);
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueFrontWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    frontWheelsSpeedMeasurements;
  const double sigma2_fl = 100;
  const double sigma2_fr = 100;
  const double e_f = 0.755;
  const double k_fl = 1.0 / 3.6 / 100.0;
  const double k_fr = 1.0 / 3.6 / 100.0;
  const double L = 2.7;
  simulateFrontWheelsSpeedMeasurements(splineTrajectory, freqW, sigma2_fl,
    sigma2_fr, e_f, L, k_fl, k_fr, T_io, trueFrontWheelsSpeedMeasurements,
    frontWheelsSpeedMeasurements);
  MeasurementsContainer<SteeringMeasurement>::Type trueSteeringMeasurements;
  MeasurementsContainer<SteeringMeasurement>::Type steeringMeasurements;
  const double sigma2_st = 10;
  const double freqSt = 30;
  const double a0 = 0;
  const double a1 = (M_PI / 180 / 10);
  const double a2 = 0;
  const double a3 = 0;
  simulateSteeringMeasurements(splineTrajectory, freqSt, sigma2_st, L, a0, a1,
    a2, a3, T_io, trueSteeringMeasurements, steeringMeasurements);
  const double sigma2_dmi = 10;
  const double freqDmi = 100;
  const double k_dmi = 1.0;
  MeasurementsContainer<ApplanixDMIMeasurement>::Type trueEncoderMeasurements;
  MeasurementsContainer<ApplanixDMIMeasurement>::Type encoderMeasurements;
  simulateDMIMeasurements(splineTrajectory, freqDmi, sigma2_dmi, e_r, T_io,
    trueEncoderMeasurements, encoderMeasurements);

  std::cout << "Building spline..." << std::endl;
  const size_t numMeasurements = navigationMeasurements.size();
  std::vector<NsecTime> timestamps;
  timestamps.reserve(numMeasurements);
  std::vector<Eigen::Vector3d> transPoses;
  transPoses.reserve(numMeasurements);
  std::vector<Eigen::Vector4d> rotPoses;
  rotPoses.reserve(numMeasurements);
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it) {
    Eigen::Vector4d quat = r2quat(
      ypr->parametersToRotationMatrix(Eigen::Vector3d(it->second.yaw,
      it->second.pitch, it->second.roll)));
    if (!rotPoses.empty()) {
      const Eigen::Vector4d lastRotPose = rotPoses.back();
      quat = bestQuat(lastRotPose, quat);
    }
    timestamps.push_back(it->first);
    rotPoses.push_back(quat);
    transPoses.push_back(Eigen::Vector3d(it->second.x, it->second.y,
      it->second.z));
  }
  const double elapsedTime = (timestamps.back() - timestamps.front()) /
    (double)NsecTimePolicy::getOne();
  const int measPerSec = std::round(numMeasurements / elapsedTime);
  int numSegments;
  if (measPerSec > knotsPerSecond)
    numSegments = std::ceil(knotsPerSecond * elapsedTime);
  else
    numSegments = numMeasurements;
  OPTBSpline<EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::CONF>::
    BSpline translationSpline(EuclideanBSpline<Eigen::Dynamic, 3,
    NsecTimePolicy>::CONF(EuclideanBSpline<Eigen::Dynamic, 3,
    NsecTimePolicy>::CONF::ManifoldConf(3), splineOrder));
  BSplineFitter<OPTBSpline<EuclideanBSpline<Eigen::Dynamic, 3,
    NsecTimePolicy>::CONF>::BSpline>::initUniformSpline(translationSpline,
    timestamps, transPoses, numSegments, lambda);
  OPTBSpline<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::CONF>::
    BSpline rotationSpline(UnitQuaternionBSpline<Eigen::Dynamic,
    NsecTimePolicy>::CONF(UnitQuaternionBSpline<Eigen::Dynamic,
    NsecTimePolicy>::CONF::ManifoldConf(), splineOrder));
  BSplineFitter<OPTBSpline<UnitQuaternionBSpline<Eigen::Dynamic,
    NsecTimePolicy>::CONF>::BSpline>::initUniformSpline(rotationSpline,
    timestamps, rotPoses, numSegments, lambda);

  std::cout << "Outputting spline data before optimization..." << std::endl;
  std::ofstream applanixSplineFile("applanix-spline.txt");
  for (auto it = timestamps.cbegin(); it != timestamps.cend(); ++it) {
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<2>(*it);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(*it);
    Eigen::Matrix3d C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression()).toRotationMatrix();
    applanixSplineFile << std::fixed << std::setprecision(18)
      << *it << " "
      << translationExpressionFactory.getValueExpression().toValue().
        transpose() << " "
      << ypr->rotationMatrixToParameters(C_wi).transpose() << " "
      << translationExpressionFactory.getValueExpression(1).toValue().
        transpose() << " "
      << -(C_wi.transpose() *
        rotationExpressionFactory.getAngularVelocityExpression().toValue()).
        transpose() << " "
      << (C_wi.transpose() *
        translationExpressionFactory.getValueExpression(2).toValue()).
        transpose()
      << std::endl;
  }

  std::cout << "Building optimization problem..." << std::endl;
  auto problem = boost::make_shared<OptimizationProblem>();
  for (size_t i = 0; i < translationSpline.numDesignVariables(); ++i) {
    translationSpline.designVariable(i)->setActive(true);
    problem->addDesignVariable(translationSpline.designVariable(i), false);
  }
  for (size_t i = 0; i < rotationSpline.numDesignVariables(); ++i) {
    rotationSpline.designVariable(i)->setActive(true);
    problem->addDesignVariable(rotationSpline.designVariable(i), false);
  }
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it) {
    ErrorTermPose::Input xm;
    xm.head<3>() = Eigen::Vector3d(it->second.x, it->second.y,
      it->second.z);
    xm.tail<3>() = Eigen::Vector3d(it->second.yaw, it->second.pitch,
      it->second.roll);
    ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
    Q(0, 0) = it->second.x_sigma2;
    Q(1, 1) = it->second.y_sigma2;
    Q(2, 2) = it->second.z_sigma2;
    Q(3, 3) = it->second.yaw_sigma2;
    Q(4, 4) = it->second.pitch_sigma2;
    Q(5, 5) = it->second.roll_sigma2;
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<0>(it->first);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<0>(it->first);
    auto e_pose = boost::make_shared<ErrorTermPose>(
      TransformationExpression(
      Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression()),
      translationExpressionFactory.getValueExpression()),
      xm, Q);
    problem->addErrorTerm(e_pose);
  }

  auto cpdv = boost::make_shared<VectorDesignVariable<12> >(
    (VectorDesignVariable<12>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr, k_dmi).finished());
  cpdv->setActive(true);
  problem->addDesignVariable(cpdv);
  auto t_io_dv = boost::make_shared<EuclideanPoint>(T_io.t());
  t_io_dv->setActive(true);
  auto C_io_dv = boost::make_shared<RotationQuaternion>(T_io.q());
  C_io_dv->setActive(true);
  RotationExpression C_io(C_io_dv);
  EuclideanExpression t_io(t_io_dv);
  problem->addDesignVariable(C_io_dv);
  problem->addDesignVariable(t_io_dv);
  NsecTime lastTimestamp = -1;
  double lastDistance = -1;
  std::ofstream errorDmiPreFile("error_dmi_pre.txt");
  std::ofstream errorDmiPreChiFile("error_dmi_pre_chi.txt");
  for (auto it = trueEncoderMeasurements.cbegin();
      it != trueEncoderMeasurements.cend(); ++it) {
    if (timestamps.front() > it->first || timestamps.back() < it->first)
      continue;
    if (lastTimestamp != -1) {
      const double displacement = it->second.signedDistanceTraveled -
        lastDistance;
      const Eigen::Matrix<double, 1, 1> meas((Eigen::Matrix<double, 1, 1>()
        << displacement / (it->first - lastTimestamp) *
        (double)NsecTimePolicy::getOne()).finished());
      auto translationExpressionFactory =
        translationSpline.getExpressionFactoryAt<1>(it->first);
      auto rotationExpressionFactory =
        rotationSpline.getExpressionFactoryAt<1>(it->first);
      auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
        rotationExpressionFactory.getValueExpression());
      auto v_ii = C_wi.inverse() *
        translationExpressionFactory.getValueExpression(1);
      auto om_ii = -(C_wi.inverse() *
        rotationExpressionFactory.getAngularVelocityExpression());
      auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
      auto om_oo = C_io.inverse() * om_ii;
      auto e_dmi = boost::make_shared<ErrorTermDMI>(v_oo, om_oo, cpdv.get(),
        meas, (Eigen::Matrix<double, 1, 1>() << sigma2_dmi).finished());
      problem->addErrorTerm(e_dmi);
      errorDmiPreChiFile << std::fixed << std::setprecision(18)
        << e_dmi->evaluateError() << std::endl;
      errorDmiPreFile << std::fixed << std::setprecision(18)
        << it->first << " " << e_dmi->error().transpose() << std::endl;
    }
    lastTimestamp = it->first;
    lastDistance = it->second.signedDistanceTraveled;
  }
  std::ofstream errorFwsPreFile("error_fws_pre.txt");
  std::ofstream errorFwsPreChiFile("error_fws_pre_chi.txt");
  for (auto it = frontWheelsSpeedMeasurements.cbegin();
      it != frontWheelsSpeedMeasurements.cend(); ++it) {
    if (timestamps.front() > it->first || timestamps.back() < it->first ||
        it->second.left == 0 || it->second.right == 0)
      continue;
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<1>(it->first);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(it->first);
    auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression());
    auto v_ii = C_wi.inverse() *
      translationExpressionFactory.getValueExpression(1);
    auto om_ii = -(C_wi.inverse() *
      rotationExpressionFactory.getAngularVelocityExpression());
    auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    auto om_oo = C_io.inverse() * om_ii;
    const double v_oo_x = v_oo.toValue()(0);
    const double om_oo_z = om_oo.toValue()(2);
    const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
    const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
    if ((v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl < 0)
      continue;
    if ((v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr < 0)
      continue;
    if (fabs(cos(phi_L)) < std::numeric_limits<double>::epsilon() ||
        fabs(cos(phi_R)) < std::numeric_limits<double>::epsilon())
      continue;
    auto e_fws = boost::make_shared<ErrorTermFws>(v_oo, om_oo, cpdv.get(),
      Eigen::Vector2d(it->second.left, it->second.right),
      (Eigen::Matrix2d() << sigma2_fl, 0, 0, sigma2_fr).finished());
    problem->addErrorTerm(e_fws);
    errorFwsPreChiFile << std::fixed << std::setprecision(18)
      << e_fws->evaluateError() << std::endl;
    errorFwsPreFile << std::fixed << std::setprecision(18)
      << it->first << " " << e_fws->error().transpose() << std::endl;
  }
  std::ofstream errorRwsPreFile("error_rws_pre.txt");
  std::ofstream errorRwsPreChiFile("error_rws_pre_chi.txt");
  for (auto it = rearWheelsSpeedMeasurements.cbegin();
      it != rearWheelsSpeedMeasurements.cend(); ++it) {
    if (timestamps.front() > it->first || timestamps.back() < it->first ||
        it->second.left == 0 || it->second.right == 0)
      continue;
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<1>(it->first);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(it->first);
    auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression());
    auto v_ii = C_wi.inverse() *
      translationExpressionFactory.getValueExpression(1);
    auto om_ii = -(C_wi.inverse() *
      rotationExpressionFactory.getAngularVelocityExpression());
    auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    auto om_oo = C_io.inverse() * om_ii;
    const double v_oo_x = v_oo.toValue()(0);
    const double om_oo_z = om_oo.toValue()(2);
    if ((v_oo_x - e_r * om_oo_z) / k_rl < 0)
      continue;
    if ((v_oo_x + e_r * om_oo_z) / k_rr < 0)
      continue;
    auto e_rws = boost::make_shared<ErrorTermRws>(v_oo, om_oo, cpdv.get(),
      Eigen::Vector2d(it->second.left, it->second.right),
      (Eigen::Matrix2d() << sigma2_rl, 0, 0, sigma2_rr).finished());
    problem->addErrorTerm(e_rws);
    errorRwsPreChiFile << std::fixed << std::setprecision(18) <<
       e_rws->evaluateError() << std::endl;
    errorRwsPreFile << std::fixed << std::setprecision(18)
      << it->first << " " << e_rws->error().transpose() << std::endl;
  }
  std::ofstream errorStPreFile("error_st_pre.txt");
  std::ofstream errorStPreChiFile("error_st_pre_chi.txt");
  for (auto it = steeringMeasurements.cbegin();
      it != steeringMeasurements.cend(); ++it) {
    if (timestamps.front() > it->first || timestamps.back() < it->first)
      continue;
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<1>(it->first);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(it->first);
    auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression());
    auto v_ii = C_wi.inverse() *
      translationExpressionFactory.getValueExpression(1);
    auto om_ii = -(C_wi.inverse() *
      rotationExpressionFactory.getAngularVelocityExpression());
    auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    auto om_oo = C_io.inverse() * om_ii;
    if (std::fabs(v_oo.toValue()(0)) < 1e-1)
      continue;
    Eigen::Matrix<double, 1, 1> meas;
    meas << it->second.value;
    auto e_st = boost::make_shared<ErrorTermSteering>(v_oo, om_oo, cpdv.get(),
      meas, (Eigen::Matrix<double, 1, 1>() << sigma2_st).finished());
    problem->addErrorTerm(e_st);
    errorStPreChiFile << std::fixed << std::setprecision(18) <<
      e_st->evaluateError() << std::endl;
    errorStPreFile << std::fixed << std::setprecision(18)
      << it->first << " " << e_st->error().transpose() << std::endl;
  }
  std::ofstream errorVmPreFile("error_vm_pre.txt");
  std::ofstream errorVmPreChiFile("error_vm_pre_chi.txt");
  for (auto it = translationSpline.begin(); it != translationSpline.end();
      ++it) {
    auto timestamp = it.getTime();
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<1>(timestamp);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(timestamp);
    auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression());
    auto v_ii = C_wi.inverse() *
      translationExpressionFactory.getValueExpression(1);
    auto om_ii = -(C_wi.inverse() *
      rotationExpressionFactory.getAngularVelocityExpression());
    auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
    auto om_oo = C_io.inverse() * om_ii;
    ErrorTermVehicleModel::Covariance Q(
      ErrorTermVehicleModel::Covariance::Zero());
    Q(0, 0) = 0.1; Q(1, 1) = 1; Q(2, 2) = 0.1; Q(3, 3) = 0.1;
    auto e_vm = boost::make_shared<ErrorTermVehicleModel>(v_oo, om_oo, Q);
    problem->addErrorTerm(e_vm);
    errorVmPreChiFile << std::fixed << std::setprecision(18) <<
      e_vm->evaluateError() << std::endl;
    errorVmPreFile << std::fixed << std::setprecision(18)
      << timestamp << " " << e_vm->error().transpose() << std::endl;
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
//  optimizer.initialize();
//  optimizer.getSolver<SparseQrLinearSystemSolver>()->buildSystem(
//    optimizer.options().nThreads, true);
//  const CompressedColumnMatrix<ssize_t>& JInit =
//    optimizer.getSolver<SparseQrLinearSystemSolver>()->getJacobianTranspose();
//  std::ofstream JInitFile("JInit.txt");
//  JInit.writeMATLAB(JInitFile);
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
  const CompressedColumnMatrix<ssize_t>& JOpt =
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getJacobianTranspose();
  const size_t numCols = JOpt.rows();
//  std::ofstream JOptFile("JOpt.txt");
//  JOpt.writeMATLAB(JOptFile);
  std::cout << "Rank: " << optimizer.getSolver<SparseQrLinearSystemSolver>()
    ->getRank() << std::endl;
  std::cout << "Rank deficiency: " << numCols -
    optimizer.getSolver<SparseQrLinearSystemSolver>()->getRank() << std::endl;
  Eigen::MatrixXd NS, CS, Sigma, SigmaP, Omega;
  marginalize(JOpt, numCols - 18, NS, CS, Sigma, SigmaP, Omega, 1e-8, 1e-9);
  std::cout << "Sigma (SVD): " << std::endl << std::fixed
    << std::setprecision(18) << Sigma.diagonal().transpose() << std::endl;
  std::cout << "SigmaP: " << std::endl << std::fixed << std::setprecision(18)
    << SigmaP.diagonal().transpose() << std::endl;
  std::cout << "NS: " << std::endl << std::fixed << std::setprecision(18)
    << NS << std::endl;
  std::cout << "Marginal rank: " << CS.cols() << std::endl;
  std::cout << "Marginal rank deficiency: " << NS.cols() << std::endl;

  std::cout << "Outputting spline data after optimization..." << std::endl;
  std::ofstream applanixSplineOptFile("applanix-spline-opt.txt");
  for (auto it = timestamps.cbegin(); it != timestamps.cend(); ++it) {
    auto translationExpressionFactory =
      translationSpline.getExpressionFactoryAt<2>(*it);
    auto rotationExpressionFactory =
      rotationSpline.getExpressionFactoryAt<1>(*it);
    Eigen::Matrix3d C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
      rotationExpressionFactory.getValueExpression()).toRotationMatrix();
    applanixSplineOptFile << std::fixed << std::setprecision(18)
      << *it << " "
      << translationExpressionFactory.getValueExpression().toValue().
        transpose() << " "
      << ypr->rotationMatrixToParameters(C_wi).transpose() << " "
      << translationExpressionFactory.getValueExpression(1).toValue().
        transpose() << " "
      << -(C_wi.transpose() *
        rotationExpressionFactory.getAngularVelocityExpression().toValue()).
        transpose() << " "
      << (C_wi.transpose() *
        translationExpressionFactory.getValueExpression(2).toValue()).
        transpose()
      << std::endl;
  }

  std::cout << "Integrating odometry..." << std::endl;
  janeth::DifferentialOdometry::Parameters params = {0.285, 0.285, 0.285, 0.285,
    e_r * 2, e_f * 2, 0.000045};
  std::ofstream diffOdoFile("diffOdo.txt");
  janeth::DifferentialOdometry odometry(params);
  bool firstPose = true;
  for (auto it = trueRearWheelsSpeedMeasurements.cbegin();
      it != trueRearWheelsSpeedMeasurements.cend(); ++it) {
    if (firstPose) {
      auto translationExpressionFactory =
        translationSpline.getExpressionFactoryAt<0>(it->first);
      auto rotationExpressionFactory =
        rotationSpline.getExpressionFactoryAt<0>(it->first);
      Eigen::Vector3d tk = translationExpressionFactory.getValueExpression().
        toValue();
      Eigen::Matrix3d C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
        rotationExpressionFactory.getValueExpression()).toRotationMatrix();
      Eigen::Vector3d C_wi_param = ypr->rotationMatrixToParameters(C_wi);
      odometry.reset(Eigen::Vector3d(tk(0), tk(1), C_wi_param(0)),
        nsecToSec(it->first));
      firstPose = false;
    }
    odometry.updateRearWheelTranslationalVelocities(it->second.left * k_rl,
      it->second.right * k_rr, nsecToSec(it->first));
    diffOdoFile << std::fixed << std::setprecision(18)
      << odometry.getPose().transpose() << std::endl;
  }
  return 0;
}
