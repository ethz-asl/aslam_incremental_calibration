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

/** \file calibrateCar.cpp
    \brief This file calibrates the car parameters from a ROS bag file.
  */

#include <iostream>
#include <vector>
#include <string>
#include <limits>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/timing/TimestampCorrector.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include <sm/kinematics/rotations.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

#include <libposlv/geo-tools/Geo.h>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/CarCalibrator.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/SteeringMeasurement.h"
#include "aslam/calibration/car/ApplanixDMIMeasurement.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;

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
  std::cout << "Processing BAG file..." << std::endl;
  const double L = 2.7; // wheelbase [m]
  const double e_r = 0.74; // half-track rear [m]
  const double e_f = 0.755; // half-track front [m]
  const double a0 = 0; // steering coefficient
  const double a1 = M_PI / 180 / 10; // steering coefficient
  const double a2 = 0; // steering coefficient
  const double a3 = 0; // steering coefficient
  const double k_rl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_rr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_dmi = 1.0; // DMI coefficient
  CarCalibrator::CalibrationDesignVariables dv;
  dv.intrinsicOdoDesignVariable =
    boost::make_shared<VectorDesignVariable<12> >(
    (VectorDesignVariable<12>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr, k_dmi).finished());
  dv.intrinsicOdoDesignVariable->setActive(true);
  dv.extrinsicOdoTranslationDesignVariable =
    boost::make_shared<EuclideanPoint>(Eigen::Vector3d(0, 0, -0.785));
  dv.extrinsicOdoTranslationDesignVariable->setActive(true);
  dv.extrinsicOdoRotationDesignVariable =
    boost::make_shared<RotationQuaternion>(
    (Eigen::Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished());
  dv.extrinsicOdoRotationDesignVariable->setActive(true);
  auto estimator = boost::make_shared<IncrementalEstimator>(1);
  CarCalibrator calibrator(estimator, dv);
  calibrator.getOptions().windowDuration = std::numeric_limits<double>::max();
  poslv::VehicleNavigationPerformanceMsgConstPtr lastVnp;
  bool firstVNS = true;
  double latRef = 0;
  double longRef = 0;
  double altRef = 0;
  size_t viewCounter = 0;
  TimestampCorrector<double> timestampCorrector1;
  TimestampCorrector<double> timestampCorrector2;
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->getTopic() == "/poslv/vehicle_navigation_performance") {
      poslv::VehicleNavigationPerformanceMsgConstPtr vnp(
        it->instantiate<poslv::VehicleNavigationPerformanceMsg>());
      lastVnp = vnp;
    }
    if (it->getTopic() == "/poslv/vehicle_navigation_solution") {
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
      calibrator.addNavigationMeasurement(data,
        round(timestampCorrector1.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())));
    }
    if (it->getTopic() == "/can_prius/front_wheels_speed") {
      can_prius::FrontWheelsSpeedMsgConstPtr fws(
        it->instantiate<can_prius::FrontWheelsSpeedMsg>());
      WheelsSpeedMeasurement data;
      data.left = fws->Left;
      data.right = fws->Right;
      calibrator.addFrontWheelsMeasurement(data, fws->header.stamp.toNSec());
    }
    if (it->getTopic() == "/can_prius/rear_wheels_speed") {
      can_prius::RearWheelsSpeedMsgConstPtr rws(
        it->instantiate<can_prius::RearWheelsSpeedMsg>());
      WheelsSpeedMeasurement data;
      data.left = rws->Left;
      data.right = rws->Right;
      calibrator.addRearWheelsMeasurement(data, rws->header.stamp.toNSec());
    }
    if (it->getTopic() == "/can_prius/steering1") {
      can_prius::Steering1MsgConstPtr st(
        it->instantiate<can_prius::Steering1Msg>());
      SteeringMeasurement data;
      data.value = st->value;
      calibrator.addSteeringMeasurement(data, st->header.stamp.toNSec());
    }
    if (it->getTopic() == "/poslv/time_tagged_dmi_data") {
      poslv::TimeTaggedDMIDataMsgConstPtr dmi(
        it->instantiate<poslv::TimeTaggedDMIDataMsg>());
      ApplanixDMIMeasurement data;
      data.signedDistanceTraveled = dmi->signedDistanceTraveled;
      data.unsignedDistanceTraveled = dmi->unsignedDistanceTraveled;
      calibrator.addDMIMeasurement(data,
        round(timestampCorrector2.correctTimestamp(
        secToNsec(dmi->timeDistance.time1), dmi->header.stamp.toNSec())));
    }
  }
  calibrator.addMeasurements();
  return 0;
}
