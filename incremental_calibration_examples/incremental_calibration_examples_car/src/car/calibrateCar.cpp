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
#include <iomanip>
#include <vector>
#include <string>

#include <Eigen/Core>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/timing/TimestampCorrector.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/BoostPropertyTree.hpp>

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
#include "aslam/calibration/car/OptimizationProblemSpline.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace sm;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <ros_bag_file> <conf_file>"
      << std::endl;
    return -1;
  }

  std::cout << "Loading configuration parameters..." << std::endl;
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[2]);
  const bool useDMI =
    propertyTree.getBool("car/calibrator/odometry/sensors/dmi/active");
  const bool useFws =
    propertyTree.getBool("car/calibrator/odometry/sensors/fws/active");
  const bool useRws =
    propertyTree.getBool("car/calibrator/odometry/sensors/rws/active");
  const bool useSt =
    propertyTree.getBool("car/calibrator/odometry/sensors/st/active");

  CarCalibrator calibrator(PropertyTree(propertyTree, "car/calibrator"));
  std::cout << "initial calibration: " << std::endl;
  auto dv = calibrator.getCalibrationDesignVariables();
//  std::cout << "odometry intrinsics: " << std::endl << std::fixed
//    << std::setprecision(18) << *dv.intrinsicOdoDesignVariable << std::endl;
  std::cout << "odometry intrinsics: " << std::endl
    << *dv.intrinsicOdoDesignVariable << std::endl;
  Eigen::MatrixXd t_io;
  dv.extrinsicOdoTranslationDesignVariable->getParameters(t_io);
  std::cout << "IMU-odometry translation: " << std::endl <<
    t_io.transpose() << std::endl;
  Eigen::MatrixXd q_io;
  dv.extrinsicOdoRotationDesignVariable->getParameters(q_io);
  EulerAnglesYawPitchRoll ypr;
  std::cout << "IMU-odometry rotation: " << std::endl <<
    ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;

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
      calibrator.addNavigationMeasurement(data,
        round(timestampCorrector1.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())));
    }
    if (it->getTopic() == "/can_prius/front_wheels_speed" && useFws) {
      can_prius::FrontWheelsSpeedMsgConstPtr fws(
        it->instantiate<can_prius::FrontWheelsSpeedMsg>());
      WheelsSpeedMeasurement data;
      data.left = fws->Left;
      data.right = fws->Right;
      calibrator.addFrontWheelsMeasurement(data, fws->header.stamp.toNSec());
    }
    if (it->getTopic() == "/can_prius/rear_wheels_speed" && useRws) {
      can_prius::RearWheelsSpeedMsgConstPtr rws(
        it->instantiate<can_prius::RearWheelsSpeedMsg>());
      WheelsSpeedMeasurement data;
      data.left = rws->Left;
      data.right = rws->Right;
      calibrator.addRearWheelsMeasurement(data, rws->header.stamp.toNSec());
    }
    if (it->getTopic() == "/can_prius/steering1" && useSt) {
      can_prius::Steering1MsgConstPtr st(
        it->instantiate<can_prius::Steering1Msg>());
      SteeringMeasurement data;
      data.value = st->value;
      calibrator.addSteeringMeasurement(data, st->header.stamp.toNSec());
    }
    if (it->getTopic() == "/poslv/time_tagged_dmi_data" && useDMI) {
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

  if (calibrator.unprocessedMeasurements())
    calibrator.addMeasurements();

  std::cout << "final calibration: " << std::endl;
//  std::cout << "odometry intrinsics: " << std::endl << std::fixed
//    << std::setprecision(18) << *dv.intrinsicOdoDesignVariable << std::endl;
  std::cout << "odometry intrinsics: " << std::endl
    << *dv.intrinsicOdoDesignVariable << std::endl;
  dv.extrinsicOdoTranslationDesignVariable->getParameters(t_io);
  std::cout << "IMU-odometry translation: " << std::endl <<
    t_io.transpose() << std::endl;
  dv.extrinsicOdoRotationDesignVariable->getParameters(q_io);
  std::cout << "IMU-odometry rotation: " << std::endl <<
    ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;

//  std::cout << "covariance: " << std::fixed << std::setprecision(18)
//    << std::endl << calibrator.getEstimator()
//      ->getMarginalizedCovariance().diagonal().transpose() << std::endl;
  auto estimator = calibrator.getEstimator();
  std::cout << "covariance: " << std::endl
    << estimator->getMarginalizedCovariance().diagonal().transpose()
    << std::endl;
  std::cout << "null space: " << std::endl
    << estimator->getMarginalizedNullSpace() << std::endl;
  std::cout << "singular values: " << std::endl
    << estimator->getSingularValues().transpose() << std::endl;
  std::cout << "memory usage [MB]: " << estimator->getMemoryUsage() /
    1024.0 / 1024.0 << std::endl;
  std::cout << "peak memory usage [MB]: "
    << estimator->getPeakMemoryUsage() / 1024.0 / 1024.0 << std::endl;

  return 0;
}
