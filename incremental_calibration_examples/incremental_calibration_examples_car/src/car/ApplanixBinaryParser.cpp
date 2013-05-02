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

#include "aslam/calibration/car/ApplanixBinaryParser.h"

#include <fstream>
#include <memory>
#include <iomanip>

#include <libposlv/sensor/BinaryLogReader.h>
#include <libposlv/types/Packet.h>
#include <libposlv/types/Group.h>
#include <libposlv/types/VehicleNavigationSolution.h>
#include <libposlv/types/VehicleNavigationPerformance.h>
#include <libposlv/sensor/Utils.h>
#include <libposlv/geo-tools/Geo.h>
#include <libposlv/exceptions/IOException.h>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    ApplanixBinaryParser::ApplanixBinaryParser(const std::string& filename) :
        _filename(filename) {
    }

    ApplanixBinaryParser::~ApplanixBinaryParser() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const std::string& ApplanixBinaryParser::getFilename() const {
      return _filename;
    }

    void ApplanixBinaryParser::setFilename(const std::string& filename) {
      _filename = filename;
    }

    size_t ApplanixBinaryParser::getNumNavigationSolution() const {
      return _navigationSolution.size();
    }

    ApplanixBinaryParser::ContainerCIt ApplanixBinaryParser::cbegin() const {
      return _navigationSolution.cbegin();
    }

    ApplanixBinaryParser::ContainerCIt ApplanixBinaryParser::cend() const {
      return _navigationSolution.cend();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void ApplanixBinaryParser::parse() {
      _navigationSolution.clear();
      std::ifstream binaryLogFile(_filename);
      if (!binaryLogFile.is_open())
        throw IOException("ApplanixBinaryParser::parse(): file opening failed");
      BinaryLogReader logReader(binaryLogFile);
      binaryLogFile.seekg (0, std::ios::end);
      const int length = binaryLogFile.tellg();
      binaryLogFile.seekg (0, std::ios::beg);
      std::shared_ptr<Packet> lastPerformance;
      double latRef = 0;
      double longRef = 0;
      double altRef = 0;
      bool firstLLA = true;
      while (binaryLogFile.tellg() != length) {
        double timestamp;
        logReader >> timestamp;
        std::shared_ptr<Packet> packet = logReader.readPacket();
        if (packet->instanceOfGroup()) {
          const Group& group = packet->groupCast();
          if (group.instanceOf<VehicleNavigationSolution>()) {
            NavigationSolution navMsg;
            if (lastPerformance == NULL)
              continue;
            else {
              const Group& perfGroup = lastPerformance->groupCast();
              const VehicleNavigationPerformance& perfMsg =
                perfGroup.typeCast<VehicleNavigationPerformance>();
                navMsg.x_sigma2 = perfMsg.mEastPositionRMSError *
                  perfMsg.mEastPositionRMSError;
                navMsg.y_sigma2 = perfMsg.mNorthPositionRMSError *
                  perfMsg.mNorthPositionRMSError;
                navMsg.z_sigma2 = perfMsg.mDownPositionRMSError *
                  perfMsg.mDownPositionRMSError;
                navMsg.roll_sigma2 = perfMsg.mRollRMSError *
                  perfMsg.mRollRMSError;
                navMsg.pitch_sigma2 = perfMsg.mPitchRMSError *
                  perfMsg.mPitchRMSError;
                navMsg.yaw_sigma2 = perfMsg.mHeadingRMSError *
                  perfMsg.mHeadingRMSError;
                navMsg.v_x_sigma2 = perfMsg.mEastPositionRMSError *
                  perfMsg.mEastPositionRMSError;
                navMsg.v_y_sigma2 = perfMsg.mNorthVelocityRMSError *
                  perfMsg.mNorthVelocityRMSError;
                navMsg.v_z_sigma2 = perfMsg.mDownVelocityRMSError *
                  perfMsg.mDownVelocityRMSError;
            }
            const VehicleNavigationSolution& msg =
              group.typeCast<VehicleNavigationSolution>();
            if (firstLLA) {
              latRef = msg.mLatitude;
              longRef = msg.mLongitude;
              altRef = msg.mAltitude;
              firstLLA = false;
            }
            double x_ecef, y_ecef, z_ecef;
            Geo::wgs84ToEcef(msg.mLatitude, msg.mLongitude, msg.mAltitude,
              x_ecef, y_ecef, z_ecef);
            Geo::ecefToEnu(x_ecef, y_ecef, z_ecef, latRef, longRef, altRef,
              navMsg.x, navMsg.y, navMsg.z);
            // Rotation matrix transforming ENU vectors into NED vectors
            Eigen::Matrix3d C_ENU_NED =
              Geo::R_ENU_NED::getInstance().getMatrix();
            const sm::kinematics::EulerAnglesYawPitchRoll ypr;
            // Rotation matrix transforming body NED to world NED
            const Eigen::Matrix3d C_w_NED_i_NED =
              ypr.parametersToRotationMatrix(
              Eigen::Vector3d(Utils::deg2rad(msg.mHeading),
              Utils::deg2rad(msg.mPitch), Utils::deg2rad(msg.mRoll)));
            // Rotation matrix transforming body ENU to world ENU
            Eigen::Matrix3d signMatrix = Eigen::Matrix3d::Identity();
            signMatrix(1, 1) = -1;
            signMatrix(2, 2) = -1;
            const Eigen::Matrix3d C_w_ENU_i_ENU = C_ENU_NED * C_w_NED_i_NED *
              signMatrix;
            const Eigen::Vector3d C_w_ENU_i_ENU_params =
              ypr.rotationMatrixToParameters(C_w_ENU_i_ENU);
            navMsg.yaw = C_w_ENU_i_ENU_params(0);
            navMsg.pitch = C_w_ENU_i_ENU_params(1);
            navMsg.roll = C_w_ENU_i_ENU_params(2);
            // linear velocity of the body in the world NED frame
            const Eigen::Vector3d v_iw_NED(msg.mNorthVelocity,
              msg.mEastVelocity, msg.mDownVelocity);
            // linear velocity of the body in the world ENU frame
            Eigen::Vector3d v_iw_ENU = C_ENU_NED * v_iw_NED;
            navMsg.v_x = v_iw_ENU(0);
            navMsg.v_y = v_iw_ENU(1);
            navMsg.v_z = v_iw_ENU(2);
            // angular velocity of the body in the body NED frame
            const Eigen::Vector3d om_ii_NED(
              Utils::deg2rad(msg.mAngularRateLong),
              Utils::deg2rad(msg.mAngularRateTrans),
              Utils::deg2rad(msg.mAngularRateDown));
            // angular velocity of the body in the body ENU frame
            Eigen::Vector3d om_ii_ENU = signMatrix * om_ii_NED;
            navMsg.om_x = om_ii_ENU(0);
            navMsg.om_y = om_ii_ENU(1);
            navMsg.om_z = om_ii_ENU(2);
            // linear acceleration of the body in the body NED frame
            const Eigen::Vector3d a_ii_NED(msg.mAccLong, msg.mAccTrans,
              msg.mAccDown);
            // linear acceleration of the body in the body ENU frame
            Eigen::Vector3d a_ii_ENU = signMatrix * a_ii_NED;
            navMsg.a_x = a_ii_ENU(0);
            navMsg.a_y = a_ii_ENU(1);
            navMsg.a_z = a_ii_ENU(2);
            navMsg.v = msg.mSpeed;
            _navigationSolution[timestamp] = navMsg;
          }
          if (group.instanceOf<VehicleNavigationPerformance>())
            lastPerformance = packet;
        }
      }
      // kick out the last measurement, problem with parsing!
      if (_navigationSolution.size() > 0) {
        auto it = _navigationSolution.end();
        --it;
        _navigationSolution.erase(it);
      }
    }

    void ApplanixBinaryParser::writeMATLAB(std::ostream& stream) const {
      for (auto it = _navigationSolution.cbegin();
          it != _navigationSolution.cend(); ++it)
        stream << std::fixed << std::setprecision(16)
          << it->first << " "
          << it->second.x << " "
          << it->second.y << " "
          << it->second.z << " "
          << it->second.yaw << " "
          << it->second.pitch << " "
          << it->second.roll << " "
          << it->second.v_x << " "
          << it->second.v_y << " "
          << it->second.v_z << " "
          << it->second.om_x << " "
          << it->second.om_y << " "
          << it->second.om_z << " "
          << it->second.a_x << " "
          << it->second.a_y << " "
          << it->second.a_z << " "
          << it->second.x_sigma2 << " "
          << it->second.y_sigma2 << " "
          << it->second.z_sigma2 << " "
          << it->second.yaw_sigma2 << " "
          << it->second.pitch_sigma2 << " "
          << it->second.roll_sigma2 << " "
          << it->second.v_x_sigma2 << " "
          << it->second.v_y_sigma2 << " "
          << it->second.v_z_sigma2 << " "
          << it->second.v << std::endl;
    }

  }
}
