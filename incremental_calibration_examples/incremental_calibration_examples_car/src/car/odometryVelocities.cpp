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

/** \file odometryVelocities.cpp
    \brief This file outputs the odometry velocities from Applanix data.
  */

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>

#include <boost/make_shared.hpp>

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
#include <bsplines/SimpleTypeTimePolicy.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>

#include <libposlv/geo-tools/Geo.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/CovarianceEstimator.h"

using namespace aslam::calibration;
using namespace sm::kinematics;
using namespace sm::timing;
using namespace bsplines;

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
  MeasurementsContainer<ApplanixNavigationMeasurement>::Type measurements;
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
      measurements.push_back(
        std::make_pair(round(timestampCorrector.correctTimestamp(
        secToNsec(vns->timeDistance.time1), vns->header.stamp.toNSec())),
        data));
    }
  }
  const size_t numMeasurements = measurements.size();
  std::vector<NsecTime> timestamps;
  timestamps.reserve(numMeasurements);
  std::vector<Eigen::Vector3d> transPoses;
  transPoses.reserve(numMeasurements);
  std::vector<Eigen::Vector4d> rotPoses;
  rotPoses.reserve(numMeasurements);
  const EulerAnglesYawPitchRoll ypr;
  for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
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

  std::cout << "Generating odometry velocities..." << std::endl;
  const Eigen::Vector3d t_io(0, 0.0, -0.785);
  const Eigen::Matrix3d C_io = Eigen::Matrix3d::Identity();
  std::ofstream odometryVelocitiesFile("odometryVelocities.txt");
  CovarianceEstimator<1> v_yEst;
  CovarianceEstimator<1> v_zEst;
  CovarianceEstimator<1> om_xEst;
  CovarianceEstimator<1> om_yEst;
  for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
    const NsecTime timestamp = it->first;
    auto transEvaluator = transSpline.getEvaluatorAt<1>(timestamp);
    auto rotEvaluator = rotSpline.getEvaluatorAt<1>(timestamp);
    const Eigen::Matrix3d C_wi = quat2r(rotEvaluator.evalD(0));
    const Eigen::Vector3d v_ii = C_wi.transpose() * transEvaluator.evalD(1);
    const Eigen::Vector3d om_ii =
      -C_wi.transpose() * rotEvaluator.evalAngularVelocity();
    const Eigen::Vector3d v_oo = C_io.transpose() *
      (v_ii + om_ii.cross(t_io));
    const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
    odometryVelocitiesFile << std::fixed << std::setprecision(18) << timestamp
      << " " << v_oo.transpose() << " " << om_oo.transpose() << std::endl;
    v_yEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
      << v_oo(1)).finished());
    v_zEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
      << v_oo(2)).finished());
    om_xEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
      << om_oo(0)).finished());
    om_yEst.addMeasurement((Eigen::Matrix<double, 1, 1>()
      << om_oo(1)).finished());
  }
  std::cout << "v_y mean: " << std::fixed << std::setprecision(18)
    << v_yEst.getMean() << std::endl;
  std::cout << "v_y variance: " << std::fixed << std::setprecision(18)
    << v_yEst.getCovariance() << std::endl;
  std::cout << "v_z mean: " << std::fixed << std::setprecision(18)
    << v_zEst.getMean() << std::endl;
  std::cout << "v_z variance: " << std::fixed << std::setprecision(18)
    << v_zEst.getCovariance() << std::endl;
  std::cout << "om_x mean: " << std::fixed << std::setprecision(18)
    << om_xEst.getMean() << std::endl;
  std::cout << "om_x variance: " << std::fixed << std::setprecision(18)
    << om_xEst.getCovariance() << std::endl;
  std::cout << "om_y mean: " << std::fixed << std::setprecision(18)
    << om_yEst.getMean() << std::endl;
  std::cout << "om_y variance: " << std::fixed << std::setprecision(18)
    << om_yEst.getCovariance() << std::endl;

  return 0;
}
