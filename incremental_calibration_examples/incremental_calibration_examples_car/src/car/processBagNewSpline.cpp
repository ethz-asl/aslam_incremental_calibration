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

/** \file processBagNewSpline.cpp
    \brief This file estimates the different covariances from a ROS bag file.
  */

#include <iostream>
#include <vector>
#include <string>
#include <limits>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/timing/TimestampCorrector.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/BSplineFitter.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

#include <libposlv/geo-tools/Geo.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/CovarianceEstimator.h"
#include "aslam/calibration/car/utils.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace bsplines;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file>" << std::endl;
    return -1;
  }
  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  topics.push_back(std::string("/can_prius/front_wheels_speed"));
  topics.push_back(std::string("/can_prius/rear_wheels_speed"));
  topics.push_back(std::string("/can_prius/steering1"));
  topics.push_back(std::string("/poslv/vehicle_navigation_solution"));
  topics.push_back(std::string("/poslv/vehicle_navigation_performance"));
  topics.push_back(std::string("/poslv/time_tagged_dmi_data"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  MeasurementsContainer<ApplanixNavigationMeasurement>::Type
    applanixNavigationMeasurements;
  bool firstVNS = true;
  double latRef = 0;
  double longRef = 0;
  double altRef = 0;
  std::cout << "Reading BAG file for Applanix navigation measurements..."
    << std::endl;
  size_t viewCounter = 0;
  TimestampCorrector<double> timestampCorrector1;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->isType<poslv::VehicleNavigationSolutionMsg>()) {
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
      applanixNavigationMeasurements.push_back(
        std::make_pair(round(timestampCorrector1.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())),
        data));
    }
  }
  std::cout << std::endl;

  std::cout << "Building spline..." << std::endl;
  const size_t numMeasurements = applanixNavigationMeasurements.size();
  std::vector<NsecTime> timestamps;
  timestamps.reserve(numMeasurements);
  std::vector<Eigen::Vector3d> transPoses;
  transPoses.reserve(numMeasurements);
  std::vector<Eigen::Vector4d> rotPoses;
  rotPoses.reserve(numMeasurements);
  const EulerAnglesYawPitchRoll ypr;
  for (auto it = applanixNavigationMeasurements.cbegin();
      it != applanixNavigationMeasurements.cend(); ++it) {
    Eigen::Vector4d quat = r2quat(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(it->second.yaw,
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
  const double lambda = 0;
  const int measPerSecDesired = 5;
  if (measPerSec > measPerSecDesired)
    numSegments = std::ceil(measPerSecDesired * elapsedTime);
  else
    numSegments = numMeasurements;
  const int transSplineOrder = 4;
  const int rotSplineOrder = 4;
  EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::TYPE transSpline(
    transSplineOrder);
  UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::TYPE rotSpline(
    rotSplineOrder);
  BSplineFitter<EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::TYPE>::
    initUniformSpline(transSpline, timestamps, transPoses, numSegments, lambda);
  BSplineFitter<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::TYPE>::
    initUniformSpline(rotSpline, timestamps, rotPoses, numSegments, lambda);

  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream applanixRawMATLABFile("applanix-raw.txt");
  for (auto it = applanixNavigationMeasurements.cbegin();
      it != applanixNavigationMeasurements.cend(); ++it)
    applanixRawMATLABFile << std::fixed << std::setprecision(18)
      << nsecToSec(it->first) << " "
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
  for (auto it = timestamps.cbegin(); it != timestamps.cend(); ++it) {
    auto transEvaluator = transSpline.getEvaluatorAt<2>(*it);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(*it);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    applanixSplineMATLABFile << std::fixed << std::setprecision(18)
      << nsecToSec(*it) << " "
      << transEvaluator.evalD(0).transpose() << " "
      << ypr.rotationMatrixToParameters(C_wi).transpose() << " "
      << transEvaluator.evalD(1).transpose() << " "
      << -(C_wi.transpose() * rotEvaluator.evalAngularVelocity()).transpose()
      << " "
      << (C_wi.transpose() * transEvaluator.evalD(2)).transpose()
      << std::endl;
  }

  std::cout << "Reading odometry data..." << std::endl;
  viewCounter = 0;
  std::ofstream canRawFwMATLABFile("can-raw-fws.txt");
  std::ofstream canPredFwMATLABFile("can-pred-fws.txt");
  std::ofstream canRawRwMATLABFile("can-raw-rws.txt");
  std::ofstream canPredRwMATLABFile("can-pred-rws.txt");
  std::ofstream canRawStMATLABFile("can-raw-st.txt");
  std::ofstream canPredStMATLABFile("can-pred-st.txt");
  std::ofstream dmiRawMATLABFile("dmi-raw.txt");
  std::ofstream dmiPredMATLABFile("dmi-pred.txt");
  const double L = 2.7; // wheelbase [m]
  const double e_r = 0.74; // half-track rear [m]
  const double e_f = 0.755; // half-track front [m]
  const double a0 = 0; // steering coefficient
  const double a1 = 1.0 / 10.0 * M_PI / 180.0; // steering coefficient
  const double k_rl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_rr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_dmi = 1.0; // DMI coefficient
  const Eigen::Vector3d t_io(0, 0, 0.785);
  const Eigen::Matrix3d C_io = ypr.parametersToRotationMatrix(
    Eigen::Vector3d(0, 0, M_PI));
  CovarianceEstimator<2> fwsCovEst;
  CovarianceEstimator<2> rwsCovEst;
  CovarianceEstimator<1> dmiCovEst;
  CovarianceEstimator<1> stCovEst;
  NsecTime lastDMITimestamp = -1;
  double lastDMIDistance = -1;
  TimestampCorrector<double> timestampCorrector2;
  const uint16_t sensorCutoff = 350;
  const double steeringMinSpeed = 1e-1;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->getTopic() == "/can_prius/front_wheels_speed") {
      can_prius::FrontWheelsSpeedMsgConstPtr fws(
        it->instantiate<can_prius::FrontWheelsSpeedMsg>());
      const NsecTime timestamp = fws->header.stamp.toNSec();
//      canRawFwMATLABFile << std::fixed << std::setprecision(18)
//        << nsecToSec(timestamp) << " " << fws->Left << " " << fws->Right
//        << std::endl;
      if (fws->Left < sensorCutoff || fws->Right < sensorCutoff ||
          timestamp < timestamps.front() || timestamp > timestamps.back())
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
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
      if (fabs(cos(phi_L)) < std::numeric_limits<double>::epsilon() ||
          fabs(cos(phi_R) < std::numeric_limits<double>::epsilon()))
        continue;
      const double predLeft = (v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl;
      const double predRight = (v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr;
      if (predLeft < 0 || predRight < 0)
        continue;
      canPredFwMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << predLeft << " " << predRight
        << std::endl;
      canRawFwMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << fws->Left << " " << fws->Right
        << std::endl;
      fwsCovEst.addMeasurement(Eigen::Vector2d(fws->Left - predLeft,
        fws->Right - predRight));
    }
    if (it->getTopic() == "/can_prius/rear_wheels_speed") {
      can_prius::RearWheelsSpeedMsgConstPtr rws(
        it->instantiate<can_prius::RearWheelsSpeedMsg>());
      const NsecTime timestamp = rws->header.stamp.toNSec();
//      canRawRwMATLABFile << std::fixed << std::setprecision(18)
//        << nsecToSec(timestamp) << " " << rws->Left << " " << rws->Right
//        << std::endl;
      if (rws->Left < sensorCutoff || rws->Right < sensorCutoff ||
          timestamp < timestamps.front() || timestamp > timestamps.back())
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
      const double predLeft = (v_oo_x - e_r * om_oo_z) / k_rl;
      const double predRight = (v_oo_x + e_r * om_oo_z) / k_rr;
      if (predLeft < 0 || predRight < 0)
        continue;
      canPredRwMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << predLeft << " " << predRight
        << std::endl;
      canRawRwMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << rws->Left << " " << rws->Right
        << std::endl;
      rwsCovEst.addMeasurement(Eigen::Vector2d(rws->Left - predLeft,
        rws->Right - predRight));
    }
    if (it->isType<can_prius::Steering1Msg>()) {
      can_prius::Steering1MsgConstPtr st(
        it->instantiate<can_prius::Steering1Msg>());
      const NsecTime timestamp = st->header.stamp.toNSec();
//      canRawStMATLABFile << std::fixed << std::setprecision(18)
//        << nsecToSec(timestamp) << " " << st->value << std::endl;
      if (timestamp < timestamps.front() || timestamp > timestamps.back())
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
      if (fabs(v_oo_x) < steeringMinSpeed)
        continue;
      const double phi = atan(L * om_oo_z / v_oo_x);
      const double predSteering = (phi - a0) / a1;
      canPredStMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << predSteering << std::endl;
      canRawStMATLABFile << std::fixed << std::setprecision(18)
        << nsecToSec(timestamp) << " " << st->value << std::endl;
      stCovEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
        << st->value - predSteering).finished());
    }
    if (it->isType<poslv::TimeTaggedDMIDataMsg>()) {
      poslv::TimeTaggedDMIDataMsgConstPtr dmi(
        it->instantiate<poslv::TimeTaggedDMIDataMsg>());
      const NsecTime timestamp = round(timestampCorrector2.correctTimestamp(
        secToNsec(dmi->timeDistance.time1), dmi->header.stamp.toNSec()));
      if (timestamp < timestamps.front() || timestamp > timestamps.back())
        continue;
      if (lastDMITimestamp != -1) {
        const double displacement = dmi->signedDistanceTraveled -
          lastDMIDistance;
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
        const double predDMI = (v_oo_x - e_r * om_oo_z) * k_dmi;
        const double measDMI = displacement / (timestamp - lastDMITimestamp) *
          (double)NsecTimePolicy::getOne();
        dmiPredMATLABFile << std::fixed << std::setprecision(18)
          << nsecToSec(timestamp) << " " << predDMI << std::endl;
        dmiRawMATLABFile << std::fixed << std::setprecision(18)
          << nsecToSec(timestamp) << " " << measDMI << std::endl;
        dmiCovEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
          << measDMI - predDMI).finished());
      }
      lastDMITimestamp = timestamp;
      lastDMIDistance = dmi->signedDistanceTraveled;
    }
  }
  std::cout << std::endl;

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
