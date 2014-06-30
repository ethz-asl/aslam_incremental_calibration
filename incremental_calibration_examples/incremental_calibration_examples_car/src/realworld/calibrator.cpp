/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

/** \file calibrator.cpp
    \brief This file estimates the odometry calibration from realworld data.
  */

#include <cmath>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/BoostPropertyTree.hpp>

#include <sm/timing/TimestampCorrector.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

#include "aslam/calibration/car/algo/CarCalibrator.h"
#include "aslam/calibration/car/algo/splinesToFile.h"
#include "aslam/calibration/car/data/WheelSpeedsMeasurement.h"
#include "aslam/calibration/car/data/SteeringMeasurement.h"
#include "aslam/calibration/car/data/DMIMeasurement.h"
#include "aslam/calibration/car/data/PoseMeasurement.h"
#include "aslam/calibration/car/data/VelocitiesMeasurement.h"
#include "aslam/calibration/car/geo/geodetic.h"

using namespace sm;
using namespace sm::timing;
using namespace sm::kinematics;
using namespace aslam::calibration;

int main(int argc, char** argv) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <conf_file>" << std::endl;
    return -1;
  }

  BoostPropertyTree config;
  config.loadXml(argv[2]);

  const bool useDMI =
    config.getBool("car/calibrator/odometry/sensors/dmi/active");
  const bool useFw =
    config.getBool("car/calibrator/odometry/sensors/fws/active");
  const bool useRw =
    config.getBool("car/calibrator/odometry/sensors/rws/active");
  const bool useSt =
    config.getBool("car/calibrator/odometry/sensors/st/active");

  CarCalibrator calibrator(PropertyTree(config, "car/calibrator"));

  std::ofstream rwDataFile("rwData.txt");
  rwDataFile << std::fixed << std::setprecision(18);
  std::ofstream fwDataFile("fwData.txt");
  fwDataFile << std::fixed << std::setprecision(18);
  std::ofstream stDataFile("stData.txt");
  stDataFile << std::fixed << std::setprecision(18);
  std::ofstream dmiDataFile("dmiData.txt");
  dmiDataFile << std::fixed << std::setprecision(18);
  std::ofstream poseDataFile("poseData.txt");
  poseDataFile << std::fixed << std::setprecision(18);
  std::ofstream velDataFile("velData.txt");
  velDataFile << std::fixed << std::setprecision(18);

  rosbag::Bag bag(argv[1]);
  std::vector<std::string> topics;
  topics.push_back(config.getString(
    "car/calibrator/odometry/sensors/fws/topic"));
  topics.push_back(config.getString(
    "car/calibrator/odometry/sensors/rws/topic"));
  topics.push_back(config.getString(
    "car/calibrator/odometry/sensors/st/topic"));
  topics.push_back(config.getString(
    "car/calibrator/odometry/sensors/dmi/topic"));
  topics.push_back(config.getString(
    "car/calibrator/applanix/vns/topic"));
  topics.push_back(config.getString(
    "car/calibrator/applanix/vnp/topic"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  TimestampCorrector<double> timestampCorrectorVns;
  TimestampCorrector<double> timestampCorrectorDmi;
  TimestampCorrector<double> timestampCorrectorFw;
  TimestampCorrector<double> timestampCorrectorRw;
  TimestampCorrector<double> timestampCorrectorSt;
  double lastDMITimestamp = -1;
  double lastDMIDistance = -1;
  bool firstVns = true;
  poslv::VehicleNavigationPerformanceMsgConstPtr lastVnp;
  Transformation m_T_e;
  const EulerAnglesYawPitchRoll ypr;
  Transformation m_T_r_0;
  for (auto it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == config.getString(
        "car/calibrator/applanix/vnp/topic")) {
      poslv::VehicleNavigationPerformanceMsgConstPtr vnp(
        it->instantiate<poslv::VehicleNavigationPerformanceMsg>());
      lastVnp = vnp;
    }
    if (it->getTopic() == config.getString(
        "car/calibrator/applanix/vns/topic")) {
      if (!lastVnp)
        continue;
      poslv::VehicleNavigationSolutionMsgConstPtr vns(
        it->instantiate<poslv::VehicleNavigationSolutionMsg>());
      double x_ecef, y_ecef, z_ecef;
      wgs84ToEcef(deg2rad(vns->latitude), deg2rad(vns->longitude),
        vns->altitude, x_ecef, y_ecef, z_ecef);
      if (firstVns) {
        m_T_e = ecef2enu(x_ecef, y_ecef, z_ecef, deg2rad(vns->latitude),
          deg2rad(vns->longitude));
      }
      PoseMeasurement pose;
      pose.m_r_mr = m_T_e * Eigen::Vector3d(x_ecef, y_ecef, z_ecef);
      const Eigen::Matrix3d l_ned_R_r = ypr.parametersToRotationMatrix(
        Eigen::Vector3d(deg2rad(vns->heading), deg2rad(vns->pitch),
        deg2rad(vns->roll)));
      const Transformation e_T_l_ned = ned2ecef(x_ecef, y_ecef, z_ecef,
        deg2rad(vns->latitude), deg2rad(vns->longitude));
      pose.m_R_r = ypr.rotationMatrixToParameters(m_T_e.C() * e_T_l_ned.C() *
        l_ned_R_r);
      pose.sigma2_m_r_mr = Eigen::Vector3d(lastVnp->northPositionRMSError *
        lastVnp->northPositionRMSError, lastVnp->eastPositionRMSError *
        lastVnp->eastPositionRMSError, lastVnp->downPositionRMSError *
        lastVnp->downPositionRMSError).asDiagonal();
      pose.sigma2_m_R_r = Eigen::Vector3d(deg2rad(lastVnp->headingRMSError) *
        deg2rad(lastVnp->headingRMSError), deg2rad(lastVnp->pitchRMSError) *
        deg2rad(lastVnp->pitchRMSError), deg2rad(lastVnp->rollRMSError) *
        deg2rad(lastVnp->rollRMSError)).asDiagonal();
      poseDataFile << vns->header.stamp.toSec() << " " <<
        pose.m_r_mr.transpose() << " " << pose.m_R_r.transpose() <<
        " " << pose.sigma2_m_r_mr.diagonal().transpose() << " " <<
        pose.sigma2_m_R_r.diagonal().transpose() << std::endl;
      if (firstVns) {
        m_T_r_0 = Transformation(
          r2quat(ypr.parametersToRotationMatrix(pose.m_R_r)), pose.m_r_mr);
        firstVns = false;
      }
      VelocitiesMeasurement vel;
      vel.r_v_mr = l_ned_R_r.transpose() * Eigen::Vector3d(vns->northVelocity,
        vns->eastVelocity, vns->downVelocity);
      vel.sigma2_r_v_mr = Eigen::Vector3d(lastVnp->northVelocityRMSError *
        lastVnp->northVelocityRMSError, lastVnp->eastVelocityRMSError *
        lastVnp->eastVelocityRMSError, lastVnp->downVelocityRMSError *
        lastVnp->downVelocityRMSError).asDiagonal();
      vel.r_om_mr = Eigen::Vector3d(deg2rad(vns->angularRateLong),
        deg2rad(vns->angularRateTrans), deg2rad(vns->angularRateDown));
      vel.sigma2_r_om_mr = Eigen::Vector3d(deg2rad(lastVnp->rollRMSError) *
        deg2rad(lastVnp->rollRMSError), deg2rad(lastVnp->pitchRMSError) *
        deg2rad(lastVnp->pitchRMSError), deg2rad(lastVnp->headingRMSError) *
        deg2rad(lastVnp->headingRMSError)).asDiagonal();
      velDataFile << vns->header.stamp.toSec() << " " <<
        vel.r_v_mr.transpose() << " " << vel.r_om_mr.transpose() <<
        " " << vel.sigma2_r_v_mr.diagonal().transpose() << " " <<
        vel.sigma2_r_om_mr.diagonal().transpose() << std::endl;
      auto timestamp = std::round(timestampCorrectorVns.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec()));
      calibrator.addPoseMeasurement(pose, timestamp);
      calibrator.addVelocitiesMeasurement(vel, timestamp);
    }
    if (it->getTopic() == config.getString(
        "car/calibrator/odometry/sensors/fws/topic") && useFw) {
      can_prius::FrontWheelsSpeedMsgConstPtr fws(
        it->instantiate<can_prius::FrontWheelsSpeedMsg>());
      WheelSpeedsMeasurement data;
      data.left = fws->Left;
      data.right = fws->Right;
      auto timestamp = std::round(timestampCorrectorFw.correctTimestamp(
        fws->header.seq, fws->header.stamp.toNSec()));
      calibrator.addFrontWheelsMeasurement(data, timestamp);
      fwDataFile << fws->header.stamp.toSec() << " " << data.left << " "
        << data.right << std::endl;
    }
    if (it->getTopic() == config.getString(
        "car/calibrator/odometry/sensors/rws/topic") && useRw) {
      can_prius::RearWheelsSpeedMsgConstPtr rws(
        it->instantiate<can_prius::RearWheelsSpeedMsg>());
      WheelSpeedsMeasurement data;
      data.left = rws->Left;
      data.right = rws->Right;
      auto timestamp = std::round(timestampCorrectorRw.correctTimestamp(
        rws->header.seq, rws->header.stamp.toNSec()));
      calibrator.addRearWheelsMeasurement(data, timestamp);
      rwDataFile << rws->header.stamp.toSec() << " " << data.left << " "
        << data.right << std::endl;
    }
    if (it->getTopic() == config.getString(
        "car/calibrator/odometry/sensors/st/topic") && useSt) {
      can_prius::Steering1MsgConstPtr st(
        it->instantiate<can_prius::Steering1Msg>());
      SteeringMeasurement data;
      data.value = st->value;
      auto timestamp = std::round(timestampCorrectorSt.correctTimestamp(
        st->header.seq, st->header.stamp.toNSec()));
      calibrator.addSteeringMeasurement(data, timestamp);
      stDataFile << st->header.stamp.toSec() << " " << data.value << std::endl;
    }
    if (it->getTopic() == config.getString(
        "car/calibrator/odometry/sensors/dmi/topic") && useDMI) {
      poslv::TimeTaggedDMIDataMsgConstPtr dmi(
        it->instantiate<poslv::TimeTaggedDMIDataMsg>());
      if (lastDMITimestamp != -1) {
        DMIMeasurement data;
        data.wheelSpeed = (dmi->signedDistanceTraveled - lastDMIDistance) /
          (dmi->timeDistance.time1 - lastDMITimestamp);
        calibrator.addDMIMeasurement(data,
          std::round(timestampCorrectorDmi.correctTimestamp(
          secToNsec(dmi->timeDistance.time1), dmi->header.stamp.toNSec())));
        dmiDataFile << dmi->header.stamp.toSec() << " " << data.wheelSpeed
          << std::endl;
      }
      lastDMITimestamp = dmi->timeDistance.time1;
      lastDMIDistance = dmi->signedDistanceTraveled;
    }
  }

  if (calibrator.unprocessedMeasurements())
    calibrator.addMeasurements();

  std::ofstream devFile("deviations.txt");
  devFile << std::fixed << std::setprecision(18);
  Eigen::VectorXd variances = calibrator.getOdometryVariablesVariance();
  devFile << "e_r: " << std::sqrt(variances(0)) << std::endl;
  devFile << "e_f: " << std::sqrt(variances(1)) << std::endl;
  devFile << "L: " << std::sqrt(variances(2)) << std::endl;
  devFile << "a0: " << std::sqrt(variances(3)) << std::endl;
  devFile << "a1: " << std::sqrt(variances(4)) << std::endl;
  devFile << "a2: " << std::sqrt(variances(5)) << std::endl;
  devFile << "a3: " << std::sqrt(variances(6)) << std::endl;
  devFile << "k_rl: " << std::sqrt(variances(7)) << std::endl;
  devFile << "k_rr: " << std::sqrt(variances(8)) << std::endl;
  devFile << "k_fl: " << std::sqrt(variances(9)) << std::endl;
  devFile << "k_fr: " << std::sqrt(variances(10)) << std::endl;
  devFile << "k_dmi: " << std::sqrt(variances(11)) << std::endl;
  devFile << "v_r_vr_1: " << std::sqrt(variances(12)) << std::endl;
  devFile << "v_r_vr_2: " << std::sqrt(variances(13)) << std::endl;
  devFile << "v_r_vr_3: " << std::sqrt(variances(14)) << std::endl;
  devFile << "v_R_r_1: " << std::sqrt(variances(15)) << std::endl;
  devFile << "v_R_r_2: " << std::sqrt(variances(16)) << std::endl;
  devFile << "v_R_r_3: " << std::sqrt(variances(17)) << std::endl;

  std::ofstream m_T_v_estFile("m_T_v_est.txt");
  m_T_v_estFile << std::fixed << std::setprecision(18);
  writeSplines(calibrator.getEstimator(), 0.01, m_T_v_estFile);

  std::ofstream infoGainHistFile("infoGainHist.txt");
  auto infoGainHist = calibrator.getInformationGainHistory();
  infoGainHistFile << std::fixed << std::setprecision(18);
  std::for_each(infoGainHist.cbegin(), infoGainHist.cend(), [&](decltype(
    *infoGainHist.cbegin()) x) {infoGainHistFile << x << std::endl;});

  std::ofstream calibHistFile("calibHist.txt");
  auto calibHist = calibrator.getOdometryVariablesHistory();
  calibHistFile << std::fixed << std::setprecision(18);
  std::for_each(calibHist.cbegin(), calibHist.cend(), [&](decltype(
    *calibHist.cbegin()) x) {calibHistFile << x.transpose() << std::endl;});

  return 0;
}
