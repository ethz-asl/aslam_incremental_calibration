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

/** \file processBagOldSpline.cpp
    \brief This file estimates the different covariances from a ROS bag file.
  */

#include <iostream>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/transformations.hpp>

#include <sm/timing/TimestampCorrector.hpp>

#include <bsplines/BSplinePose.hpp>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

#include <libposlv/geo-tools/Geo.h>

#include "aslam/calibration/car/CarCalibrator.h"
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
  CarCalibrator::ApplanixNavigationMeasurements applanixNavigationMeasurements;
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
      double x_enu, y_enu, z_enu;
      Geo::ecefToEnu(x_ecef, y_ecef, z_ecef, latRef, longRef, altRef, x_enu,
        y_enu, z_enu);
      CarCalibrator::ApplanixNavigationMeasurement data;
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
      applanixNavigationMeasurements.push_back(
        std::make_pair(timestampCorrector1.correctTimestamp(
        vns->timeDistance.time1, vns->header.stamp.toSec()), data));
    }
  }
  std::cout << std::endl;
  std::cout << "Building spline..." << std::endl;
  const size_t numMeasurements = applanixNavigationMeasurements.size();
  Eigen::VectorXd timestamps(numMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
  const EulerAnglesYawPitchRoll ypr;
  auto rv = boost::make_shared<RotationVector>();
  std::ofstream crvFile("crv.txt");
  for (size_t i = 0; i < numMeasurements; ++i) {
    Eigen::Vector3d crv = rv->rotationMatrixToParameters(
      ypr.parametersToRotationMatrix(Eigen::Vector3d(
      applanixNavigationMeasurements[i].second.yaw,
      applanixNavigationMeasurements[i].second.pitch,
      applanixNavigationMeasurements[i].second.roll)));
    if (i > 0) {
      Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
      crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
      crvFile << crv.transpose() << std::endl;
    }
    timestamps(i) = applanixNavigationMeasurements[i].first;
    Eigen::Matrix<double, 6, 1> pose;
    pose << applanixNavigationMeasurements[i].second.x,
      applanixNavigationMeasurements[i].second.y,
      applanixNavigationMeasurements[i].second.z,
      crv;
    poses.col(i) = pose;
  }
  const double elapsedTime =
    timestamps[numMeasurements - 1] - timestamps[0];
  const int measPerSec = numMeasurements / elapsedTime;
  int numSegments;
  const double lambda = 1e-1;
  const int measPerSecDesired = 5;
  if (measPerSec > measPerSecDesired)
    numSegments = measPerSecDesired * elapsedTime;
  else
    numSegments = numMeasurements;
  BSplinePose bspline(4, rv);
  bspline.initPoseSplineSparse(timestamps, poses, numSegments, lambda);
  std::cout << "Outputting raw data to MATLAB..." << std::endl;
  std::ofstream applanixRawMATLABFile("applanix-raw.txt");
  for (auto it = applanixNavigationMeasurements.cbegin();
      it != applanixNavigationMeasurements.cend(); ++it)
    applanixRawMATLABFile << std::fixed << std::setprecision(16)
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
    applanixSplineMATLABFile << std::fixed << std::setprecision(16)
      << timestamps(i) << " "
      << bspline.position(timestamps(i)).transpose() << " "
      << ypr.rotationMatrixToParameters(
        bspline.orientation(timestamps(i))).transpose() << " "
      << bspline.linearVelocity(timestamps(i)).transpose() << " "
      << bspline.angularVelocityBodyFrame(timestamps(i)).transpose() << " "
      << bspline.linearAccelerationBodyFrame(timestamps(i)).transpose()
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
  double lastDMITimestamp = -1;
  double lastDMIDistance = -1;
  const double L = 2.7; // wheelbase [m]
  const double e_r = 0.74; // half-track rear [m]
  const double e_f = 0.755; // half-track front [m]
  const double a1 = 1.0 / (M_PI / 180 / 10); // steering coefficient
  const double k_rl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_rr = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fl = 1.0 / 3.6 / 100.0; // wheel coefficient
  const double k_fr = 1.0 / 3.6 / 100.0; // wheel coefficient
  Eigen::Vector3d t_io(0, 0.0, -0.785);
  Eigen::Matrix3d C_io = Eigen::Matrix3d::Identity();
  TimestampCorrector<double> timestampCorrector2;
  Eigen::Matrix4d T_wi_km1;
  const Eigen::Matrix4d T_io = rt2Transform(C_io, t_io);
  for (auto it = view.begin(); it != view.end(); ++it) {
    std::cout << std::fixed << std::setw(3)
      << viewCounter++ / (double)view.size() * 100 << " %" << '\r';
    if (it->getTopic() == "/can_prius/front_wheels_speed") {
      can_prius::FrontWheelsSpeedMsgConstPtr fws(
        it->instantiate<can_prius::FrontWheelsSpeedMsg>());
      const double timestamp = fws->header.stamp.toSec();
      canRawFwMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << fws->Left << " " << fws->Right << std::endl;
      if (fws->Left == 0 || fws->Right == 0 || timestamp < timestamps(0) ||
          timestamp > timestamps(numMeasurements - 1))
        continue;
      const Eigen::Vector3d v_iw(bspline.linearVelocity(timestamp));
      const Eigen::Vector3d om_ii(bspline.angularVelocityBodyFrame(timestamp));
      const Eigen::Matrix3d C_wi(bspline.orientation(timestamp));
      const Eigen::Vector3d v_ii = C_wi.transpose() * v_iw;
      const Eigen::Vector3d v_oo = C_io.transpose() * (v_ii + om_ii.cross(t_io));
      const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
      const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
      const double predLeft = (v_oo_x - e_f * om_oo_z) / cos(phi_L) / k_fl;
      const double predRight = (v_oo_x + e_f * om_oo_z) / cos(phi_R) / k_fr;
      canPredFwMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << predLeft << " " << predRight << std::endl;
    }
    if (it->getTopic() == "/can_prius/rear_wheels_speed") {
      can_prius::RearWheelsSpeedMsgConstPtr rws(
        it->instantiate<can_prius::RearWheelsSpeedMsg>());
      const double timestamp = rws->header.stamp.toSec();
      canRawRwMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << rws->Left << " " << rws->Right << std::endl;
      if (rws->Left == 0 || rws->Right == 0 || timestamp < timestamps(0) ||
          timestamp > timestamps(numMeasurements - 1))
        continue;
      const Eigen::Vector3d v_iw(bspline.linearVelocity(timestamp));
      const Eigen::Vector3d om_ii(bspline.angularVelocityBodyFrame(timestamp));
      const Eigen::Matrix3d C_wi(bspline.orientation(timestamp));
      const Eigen::Vector3d v_ii = C_wi.transpose() * v_iw;
      const Eigen::Vector3d v_oo = C_io.transpose() * (v_ii + om_ii.cross(t_io));
      const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      const double predLeft = (v_oo_x - e_r * om_oo_z) / k_rl;
      const double predRight = (v_oo_x + e_r * om_oo_z) / k_rr;
      canPredRwMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << predLeft << " " << predRight << std::endl;
    }
    if (it->isType<can_prius::Steering1Msg>()) {
      can_prius::Steering1MsgConstPtr st(
        it->instantiate<can_prius::Steering1Msg>());
      const double timestamp = st->header.stamp.toSec();
      canRawStMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << st->value << std::endl;
      if (timestamp < timestamps(0) ||
          timestamp > timestamps(numMeasurements - 1))
        continue;
      const Eigen::Vector3d v_iw(bspline.linearVelocity(timestamp));
      const Eigen::Vector3d om_ii(bspline.angularVelocityBodyFrame(timestamp));
      const Eigen::Matrix3d C_wi(bspline.orientation(timestamp));
      const Eigen::Vector3d v_ii = C_wi.transpose() * v_iw;
      const Eigen::Vector3d v_oo = C_io.transpose() * (v_ii + om_ii.cross(t_io));
      const Eigen::Vector3d om_oo = C_io.transpose() * om_ii;
      const double v_oo_x = v_oo(0);
      const double om_oo_z = om_oo(2);
      if (std::fabs(v_oo_x) < 1e-1)
        continue;
      const double predSteering = atan(L * om_oo_z / v_oo_x);
      canPredStMATLABFile << std::fixed << std::setprecision(16)
        << timestamp << " " << predSteering * a1 << std::endl;
    }
    if (it->isType<poslv::TimeTaggedDMIDataMsg>()) {
      poslv::TimeTaggedDMIDataMsgConstPtr dmi(
        it->instantiate<poslv::TimeTaggedDMIDataMsg>());
      const double timestamp = timestampCorrector2.correctTimestamp(
        dmi->timeDistance.time1, dmi->header.stamp.toSec());
      if (timestamp < timestamps(0) ||
          timestamp > timestamps(numMeasurements - 1))
        continue;
      const Eigen::Matrix3d C_wi = bspline.orientation(timestamp);
      const Eigen::Vector3d t_wi = bspline.position(timestamp);
      const Eigen::Matrix4d T_wi_k = rt2Transform(C_wi, t_wi);
      if (lastDMITimestamp != -1) {
        const double displacement = dmi->signedDistanceTraveled -
          lastDMIDistance;
        dmiRawMATLABFile << std::fixed << std::setprecision(16)
          << timestamp << " " << displacement << std::endl;
        const Eigen::Matrix4d T_o_km1_o_k = T_io.inverse() *
          T_wi_km1.inverse() * T_wi_k * T_io;
        const Eigen::Vector3d t_o_km1_o_k = transform2rho(T_o_km1_o_k);
        const Eigen::Matrix3d C_o_km1_o_k = transform2C(T_o_km1_o_k);
        const double v_oo_x = t_o_km1_o_k(0);
        const double om_oo_z = (ypr.rotationMatrixToParameters(C_o_km1_o_k))(0);
        const double predLeft = (v_oo_x - e_r * om_oo_z);
        dmiPredMATLABFile << std::fixed << std::setprecision(16)
          << timestamp << " " << predLeft << std::endl;
      }
      lastDMITimestamp = timestamp;
      lastDMIDistance = dmi->signedDistanceTraveled;
      T_wi_km1 = T_wi_k;
    }
  }
  return 0;
}
