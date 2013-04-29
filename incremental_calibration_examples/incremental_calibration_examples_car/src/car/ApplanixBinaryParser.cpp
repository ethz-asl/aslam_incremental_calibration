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

#include <Eigen/Core>

#include <libposlv/sensor/BinaryLogReader.h>
#include <libposlv/types/Packet.h>
#include <libposlv/types/Group.h>
#include <libposlv/types/VehicleNavigationSolution.h>
#include <libposlv/types/VehicleNavigationPerformance.h>
#include <libposlv/sensor/Utils.h>

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

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void ApplanixBinaryParser::parse() {
      _navigationSolution.clear();
      std::ifstream binaryLogFile(_filename);
      BinaryLogReader logReader(binaryLogFile);
      binaryLogFile.seekg (0, std::ios::end);
      const int length = binaryLogFile.tellg();
      binaryLogFile.seekg (0, std::ios::beg);
      std::shared_ptr<Packet> lastPerformance;
      while (binaryLogFile.tellg() != length) {
        double timestamp;
        logReader >> timestamp;
        std::shared_ptr<Packet> packet = logReader.readPacket();
        if (packet->instanceOfGroup()) {
          const Group& group = packet->groupCast();
          if (group.instanceOf<VehicleNavigationSolution>()) {
            const VehicleNavigationSolution& msg =
              group.typeCast<VehicleNavigationSolution>();
            NavigationSolution navMsg;
            if (lastPerformance == NULL) {
              navMsg.x_sigma2 = 1e-3;
              navMsg.y_sigma2 = 1e-3;
              navMsg.z_sigma2 = 1e-3;
              navMsg.roll_sigma2 = 1e-3;
              navMsg.pitch_sigma2 = 1e-3;
              navMsg.yaw_sigma2 = 1e-3;
              navMsg.v_x_sigma2 = 1e-3;
              navMsg.v_y_sigma2 = 1e-3;
              navMsg.v_z_sigma2 = 1e-3;
            }
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
            double east, north, height;
            Utils::WGS84ToLV03(msg.mLatitude, msg.mLongitude, msg.mAltitude,
              east, north, height);
            navMsg.x = east;
            navMsg.y = north;
            navMsg.z = height;
            Eigen::Matrix3d R_ENU_NED;
            R_ENU_NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;
            const sm::kinematics::EulerAnglesYawPitchRoll ypr;
            const Eigen::Matrix3d R_NED = ypr.parametersToRotationMatrix(
              Eigen::Vector3d(Utils::deg2rad(msg.mHeading),
              Utils::deg2rad(msg.mPitch), Utils::deg2rad(msg.mRoll)));
            const Eigen::Matrix3d R_ENU = R_ENU_NED * R_NED;
            const Eigen::Vector3d R_ENU_params =
              ypr.rotationMatrixToParameters(R_ENU);
            navMsg.yaw = R_ENU_params(0);
            navMsg.pitch = R_ENU_params(1);
            navMsg.roll = M_PI + R_ENU_params(2);
            const Eigen::Vector3d v_NED(msg.mNorthVelocity, msg.mEastVelocity,
              msg.mDownVelocity);
            Eigen::Vector3d v_ENU = R_ENU_NED * v_NED;
            navMsg.v_x = v_ENU(0);
            navMsg.v_y = v_ENU(1);
            navMsg.v_z = v_ENU(2);
            const Eigen::Vector3d om_NED(msg.mAngularRateLong,
              msg.mAngularRateTrans, msg.mAngularRateDown);
            Eigen::Vector3d om_ENU = R_ENU_NED * om_NED;
            navMsg.om_x = om_ENU(0);
            navMsg.om_y = om_ENU(1);
            navMsg.om_z = om_ENU(2);
            const Eigen::Vector3d a_NED(msg.mAccLong, msg.mAccTrans,
              msg.mAccDown);
            Eigen::Vector3d a_ENU = R_ENU_NED * a_NED;
            navMsg.a_x = a_ENU(0);
            navMsg.a_y = a_ENU(1);
            navMsg.a_z = a_ENU(2);
            navMsg.v = msg.mSpeed;
            _navigationSolution[timestamp] = navMsg;
          }
          if (group.instanceOf<VehicleNavigationPerformance>())
            lastPerformance = packet;
        }
      }
    }

    void ApplanixBinaryParser::writeMATLAB(std::ostream& stream) const {
      for (auto it = _navigationSolution.cbegin();
          it != _navigationSolution.cend(); ++it)
        stream
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
