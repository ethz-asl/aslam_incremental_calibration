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

/** \file simulateCarOdometry.cpp
    \brief This file simulates car data and optimizes it.
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
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/BSplineFitter.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/SimpleTypeTimePolicy.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

#include <libposlv/geo-tools/Geo.h>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/algorithms/matrixOperations.h>

#include <aslam/DiscreteTrajectory.hpp>
#include <aslam/SplineTrajectory.hpp>

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
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/CovarianceEstimator.h"

using namespace aslam;
using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace bsplines;
using namespace aslam::splines;
using namespace aslam::backend;

struct NsecTimePolicy :
  public SimpleTypeTimePolicy<NsecTime> {
  inline static NsecTime getOne() {
    return NsecTime(1e9);
  }
};

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
    }
  }

  std::cout << "Simulating data..." << std::endl;
  DiscreteTrajectory discreteTrajectory;
  generateTrajectory(navigationMeasurements, discreteTrajectory);
  const double lambda = 0;
  const int knotsPerSecond = 5;
  const int splineOrder = 4;
  SplineTrajectory splineTrajectory(splineOrder);
  splineTrajectory.initialize(discreteTrajectory, knotsPerSecond, lambda);
  Trajectory::NsecTimeList times;
  discreteTrajectory.getSupportTimes(times);
  const double freqW = 60;
  const double sigma2_rl = 9.339204354469231e+02;
  const double sigma2_rr = 1.099101100732849e+03;
  const double e_r = 0.74;
  const double k_rl = 1.0 / 3.6 / 100.0;
  const double k_rr = 1.0 / 3.6 / 100.0;
  const Transformation T_io(r2quat(Eigen::Matrix3d::Identity()),
    Eigen::Vector3d(0, 0.0, -0.785));
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
  const double sigma2_fl = 1.322034505069044e+03;
  const double sigma2_fr = 1.365297094304437e+03;
  const double e_f = 0.755;
  const double k_fl = 1.0 / 3.6 / 100.0;
  const double k_fr = 1.0 / 3.6 / 100.0;
  const double L = 2.7;
  simulateFrontWheelsSpeedMeasurements(splineTrajectory, freqW, sigma2_fl,
    sigma2_fr, e_f, L, k_fl, k_fr, T_io, trueFrontWheelsSpeedMeasurements,
    frontWheelsSpeedMeasurements);
  MeasurementsContainer<SteeringMeasurement>::Type trueSteeringMeasurements;
  MeasurementsContainer<SteeringMeasurement>::Type steeringMeasurements;
  const double sigma2_st = 1;
  const double freqSt = 30;
  const double a0 = 0;
  const double a1 = (M_PI / 180 / 10);
  const double a2 = 0;
  const double a3 = 0;
  simulateSteeringMeasurements(splineTrajectory, freqSt, sigma2_st, L, a0, a1,
    a2, a3, T_io, trueSteeringMeasurements, steeringMeasurements);
  const double sigma2_dmi = 1.080401896974195e-06;
  const double freqDmi = 100;
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
  const EulerAnglesYawPitchRoll ypr;
  for (size_t i = 0; i < numMeasurements; ++i) {
    Eigen::Vector4d quat = r2quat(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(
      navigationMeasurements[i].second.yaw,
      navigationMeasurements[i].second.pitch,
      navigationMeasurements[i].second.roll)));
    if (i > 0) {
      const Eigen::Vector4d lastRotPose = rotPoses.back();
      quat = bestQuat(lastRotPose, quat);
    }
    timestamps.push_back(navigationMeasurements[i].first);
    rotPoses.push_back(quat);
    transPoses.push_back(Eigen::Vector3d(
      navigationMeasurements[i].second.x,
      navigationMeasurements[i].second.y,
      navigationMeasurements[i].second.z));
  }
  const double elapsedTime = (timestamps[numMeasurements - 1] - timestamps[0]) /
    (double)::NsecTimePolicy::getOne();
  const int measPerSec = std::round(numMeasurements / elapsedTime);
  int numSegments;
  if (measPerSec > knotsPerSecond)
    numSegments = std::ceil(knotsPerSecond * elapsedTime);
  else
    numSegments = numMeasurements;
  EuclideanBSpline<Eigen::Dynamic, 3, ::NsecTimePolicy>::TYPE transSpline(
    splineOrder);
  UnitQuaternionBSpline<Eigen::Dynamic, ::NsecTimePolicy>::TYPE rotSpline(
    splineOrder);
  BSplineFitter<EuclideanBSpline<Eigen::Dynamic, 3, ::NsecTimePolicy>::TYPE>::
    initUniformSplineSparse(transSpline, timestamps, transPoses, numSegments,
    lambda);
  BSplineFitter<UnitQuaternionBSpline<Eigen::Dynamic, ::NsecTimePolicy>::TYPE>::
    initUniformSplineSparse(rotSpline, timestamps, rotPoses, numSegments,
    lambda);
  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream applanixRawMATLABFile("applanix-raw.txt");
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it)
    applanixRawMATLABFile << std::fixed << std::setprecision(18)
      << it->first << " "
      << it->second.x << " " << it->second.y << " " << it->second.z << " "
      << it->second.yaw << " " << it->second.pitch << " "
      << it->second.roll << " "
      << it->second.v_x << " " << it->second.v_y << " " << it->second.v_z << " "
      << it->second.om_x << " " << it->second.om_y << " "
      << it->second.om_z << " "
      << it->second.a_x << " " << it->second.a_y << " " << it->second.a_z
      << std::endl;
  std::cout << "Outputting spline data to MATLAB..." << std::endl;
  std::ofstream applanixSplineMATLABFile("applanix-spline.txt");
  for (size_t i = 0; i < numMeasurements; ++i) {
    auto transEvaluator = transSpline.getEvaluatorAt<2>(timestamps[i]);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(timestamps[i]);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    applanixSplineMATLABFile << std::fixed << std::setprecision(18)
      << timestamps[i] << " "
      << transEvaluator.evalD(0).transpose() << " "
      << ypr.rotationMatrixToParameters(C_wi).transpose() << " "
      << transEvaluator.evalD(1).transpose() << " "
      << -(C_wi.transpose() * rotEvaluator.evalAngularVelocity()).transpose()
      << " "
      << (C_wi.transpose() * transEvaluator.evalD(2)).transpose()
      << std::endl;
  }
  std::cout << "Reading odometry data..." << std::endl;
  std::ofstream canRawFwMATLABFile("can-raw-fws.txt");
  std::ofstream canPredFwMATLABFile("can-pred-fws.txt");
  std::ofstream canRawRwMATLABFile("can-raw-rws.txt");
  std::ofstream canPredRwMATLABFile("can-pred-rws.txt");
  std::ofstream canRawStMATLABFile("can-raw-st.txt");
  std::ofstream canPredStMATLABFile("can-pred-st.txt");
  std::ofstream dmiRawMATLABFile("dmi-raw.txt");
  std::ofstream dmiPredMATLABFile("dmi-pred.txt");
  const Eigen::Vector3d t_io(0, 0.0, -0.785);
  const Eigen::Matrix3d C_io = Eigen::Matrix3d::Identity();
  CovarianceEstimator<2> fwsCovEst;
  CovarianceEstimator<2> rwsCovEst;
  CovarianceEstimator<1> dmiCovEst;
  CovarianceEstimator<1> stCovEst;
  double lastDMITimestamp = -1;
  double lastDMIDistance = -1;
  Eigen::Matrix4d T_wi_km1;
  for (auto it = frontWheelsSpeedMeasurements.cbegin();
      it != frontWheelsSpeedMeasurements.cend(); ++it) {
    const NsecTime timestamp = it->first;
    canRawFwMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << it->second.left << " " << it->second.right
      << std::endl;
    auto transEvaluator = transSpline.getEvaluatorAt<1>(timestamp);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(timestamp);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    const Eigen::Vector3d v_ii = C_wi.transpose() * transEvaluator.evalD(1);
    const Eigen::Vector3d om_ii =
      -C_wi.transpose() * rotEvaluator.evalAngularVelocity();
    const Eigen::Vector3d v_oo = C_io.transpose() *
      (v_ii + om_ii.cross(t_io));
    const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
    const double v_oo_x = v_oo(0);
    const double om_oo_z = om_oo(2);
    const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
    const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
    const uint16_t predLeft = fabs(round((v_oo_x - e_f * om_oo_z) / cos(phi_L) /
      k_fl));
    const uint16_t predRight = fabs(round((v_oo_x + e_f * om_oo_z) /
      cos(phi_R) / k_fr));
    canPredFwMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << predLeft << " " << predRight << std::endl;
    fwsCovEst.addMeasurement(Eigen::Vector2d(it->second.left - predLeft,
      it->second.right - predRight));
  }
  for (auto it = rearWheelsSpeedMeasurements.cbegin();
      it != rearWheelsSpeedMeasurements.cend(); ++it) {
    const NsecTime timestamp = it->first;
    canRawRwMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << it->second.left << " " << it->second.right
      << std::endl;
    auto transEvaluator = transSpline.getEvaluatorAt<1>(timestamp);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(timestamp);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    const Eigen::Vector3d v_ii = C_wi.transpose() * transEvaluator.evalD(1);
    const Eigen::Vector3d om_ii =
      -C_wi.transpose() * rotEvaluator.evalAngularVelocity();
    const Eigen::Vector3d v_oo = C_io.transpose() *
      (v_ii + om_ii.cross(t_io));
    const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
    const double v_oo_x = v_oo(0);
    const double om_oo_z = om_oo(2);
    const uint16_t predLeft = fabs(round((v_oo_x - e_r * om_oo_z) / k_rl));
    const uint16_t predRight = fabs(round((v_oo_x + e_r * om_oo_z) / k_rr));
    canPredRwMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << predLeft << " " << predRight << std::endl;
    rwsCovEst.addMeasurement(Eigen::Vector2d(it->second.left - predLeft,
      it->second.right - predRight));
  }
  for (auto it = steeringMeasurements.cbegin();
      it != steeringMeasurements.cend(); ++it) {
    const NsecTime timestamp = it->first;
    canRawStMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << it->second.value << std::endl;
    if (timestamp < timestamps[0] ||
        timestamp > timestamps[numMeasurements - 1])
      continue;
    auto transEvaluator = transSpline.getEvaluatorAt<1>(timestamp);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(timestamp);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    const Eigen::Vector3d v_ii = C_wi.transpose() * transEvaluator.evalD(1);
    const Eigen::Vector3d om_ii =
      -C_wi.transpose() * rotEvaluator.evalAngularVelocity();
    const Eigen::Vector3d v_oo = C_io.transpose() *
      (v_ii + om_ii.cross(t_io));
    const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
    const double v_oo_x = v_oo(0);
    const double om_oo_z = om_oo(2);
    if (std::fabs(v_oo_x) < 1e-1)
      continue;
    const double phi = atan(L * om_oo_z / v_oo_x);
    const int16_t predSteering = round((phi - a0) / a1);
    canPredStMATLABFile << std::fixed << std::setprecision(18)
      << timestamp << " " << predSteering << std::endl;
    stCovEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
      << it->second.value - predSteering).finished());
   }
  for (auto it = encoderMeasurements.cbegin();
      it != encoderMeasurements.cend(); ++it) {
    const NsecTime timestamp = it->first;
    if (timestamp < timestamps[0] ||
        timestamp > timestamps[numMeasurements - 1])
      continue;
    auto transEvaluator = transSpline.getEvaluatorAt<0>(timestamp);
    auto rotEvaluator = rotSpline.getEvaluatorAt<0>(timestamp);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    const Eigen::Vector3d t_wi = transEvaluator.evalD(0);
    const Eigen::Matrix4d T_wi_k = rt2Transform(C_wi, t_wi);
    if (lastDMITimestamp != -1) {
      const double displacement = it->second.signedDistanceTraveled -
        lastDMIDistance;
      dmiRawMATLABFile << std::fixed << std::setprecision(18)
        << timestamp << " " << displacement << std::endl;
      const Eigen::Matrix4d T_o_km1_o_k = T_io.T().inverse() *
        T_wi_km1.inverse() * T_wi_k * T_io.T();
      const Eigen::Vector3d t_o_km1_o_k = transform2rho(T_o_km1_o_k);
      const Eigen::Matrix3d C_o_km1_o_k = transform2C(T_o_km1_o_k);
      const double v_oo_x = t_o_km1_o_k(0);
      const double om_oo_z = (ypr.rotationMatrixToParameters(C_o_km1_o_k))(0);
      const double predLeft = (v_oo_x - e_r * om_oo_z);
      dmiPredMATLABFile << std::fixed << std::setprecision(18)
        << timestamp << " " << predLeft << std::endl;
      dmiCovEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
        << displacement - predLeft).finished());
    }
    lastDMITimestamp = timestamp;
    lastDMIDistance = it->second.signedDistanceTraveled;
    T_wi_km1 = T_wi_k;
   }
  std::cout << "Front wheels speed mean: " << std::endl
    << fwsCovEst.getMean() << std::endl;
  std::cout << "Front wheels speed covariance: " << std::endl
    << fwsCovEst.getCovariance() << std::endl;
  std::cout << "Front wheels speed chi-square estimated mean: " << std::endl
    << fwsCovEst.getEstChiSquaredMean() << std::endl;
  std::cout << "Front wheels speed chi-square estimated variance: " << std::endl
    << fwsCovEst.getEstChiSquaredVariance() << std::endl;
  std::cout << "Front wheels speed chi-square mean: " << std::endl
    << fwsCovEst.getChiSquaredMean() << std::endl;
  std::cout << "Front wheels speed chi-square variance: " << std::endl
    << fwsCovEst.getChiSquaredVariance() << std::endl;
  std::cout << "Rear wheels speed mean: " << std::endl
    << rwsCovEst.getMean() << std::endl;
  std::cout << "Rear wheels speed covariance: " << std::endl
    << rwsCovEst.getCovariance() << std::endl;
  std::cout << "Rear wheels speed chi-square estimated mean: " << std::endl
    << rwsCovEst.getEstChiSquaredMean() << std::endl;
  std::cout << "Rear wheels speed chi-square estimated variance: " << std::endl
    << rwsCovEst.getEstChiSquaredVariance() << std::endl;
  std::cout << "Rear wheels speed chi-square mean: " << std::endl
    << rwsCovEst.getChiSquaredMean() << std::endl;
  std::cout << "Rear wheels speed chi-square variance: " << std::endl
    << rwsCovEst.getChiSquaredVariance() << std::endl;
  std::cout << "Steering mean: " << std::endl
    << stCovEst.getMean() << std::endl;
  std::cout << "Steering covariance: " << std::endl
    << stCovEst.getCovariance() << std::endl;
  std::cout << "Steering chi-square estimated mean: " << std::endl
    << stCovEst.getEstChiSquaredMean() << std::endl;
  std::cout << "Steering chi-square estimated variance: " << std::endl
    << stCovEst.getEstChiSquaredVariance() << std::endl;
  std::cout << "Steering chi-square mean: " << std::endl
    << stCovEst.getChiSquaredMean() << std::endl;
  std::cout << "Steering chi-square variance: " << std::endl
    << stCovEst.getChiSquaredVariance() << std::endl;
  std::cout << "DMI mean: " << std::endl
    << dmiCovEst.getMean() << std::endl;
  std::cout << "DMI covariance: " << std::endl
    << dmiCovEst.getCovariance() << std::endl;
  std::cout << "DMI chi-square estimated mean: " << std::endl
    << dmiCovEst.getEstChiSquaredMean() << std::endl;
  std::cout << "DMI chi-square estimated variance: " << std::endl
    << dmiCovEst.getEstChiSquaredVariance() << std::endl;
  std::cout << "DMI chi-square mean: " << std::endl
    << dmiCovEst.getChiSquaredMean() << std::endl;
  std::cout << "DMI chi-square variance: " << std::endl
    << dmiCovEst.getChiSquaredVariance() << std::endl;

  return 0;
}
