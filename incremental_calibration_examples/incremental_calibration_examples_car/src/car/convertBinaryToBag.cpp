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

/** \file convertBinaryToBag.cpp
    \brief This file converts old binary format to BAG format.
  */

#include <iostream>
#include <fstream>
#include <memory>

#include <libposlv/exceptions/IOException.h>
#include <libposlv/sensor/BinaryLogReader.h>
#include <libposlv/types/Packet.h>
#include <libposlv/types/Group.h>
#include <libposlv/types/VehicleNavigationSolution.h>
#include <libposlv/types/VehicleNavigationPerformance.h>
#include <libposlv/types/TimeTaggedDMIData.h>

#include <libcan-prius/base/BinaryStreamReader.h>
#include <libcan-prius/types/PRIUSMessage.h>
#include <libcan-prius/types/FrontWheelsSpeed.h>
#include <libcan-prius/types/RearWheelsSpeed.h>
#include <libcan-prius/types/Steering1.h>
#include <libcan-prius/base/Factory.h>

#include <rosbag/bag.h>

#include <poslv/VehicleNavigationSolutionMsg.h>
#include <poslv/VehicleNavigationPerformanceMsg.h>
#include <poslv/TimeTaggedDMIDataMsg.h>

#include <can_prius/FrontWheelsSpeedMsg.h>
#include <can_prius/RearWheelsSpeedMsg.h>
#include <can_prius/Steering1Msg.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
      << " <applanix-binary-log> <can-binary-log>" << std::endl;
    return -1;
  }

  rosbag::Bag bag("binary.bag", rosbag::bagmode::Write);

  std::ifstream applanixBinaryLogFile(argv[1]);
  if (!applanixBinaryLogFile.is_open())
    throw IOException("File opening failed");
  BinaryLogReader applanixLogReader(applanixBinaryLogFile);
  applanixBinaryLogFile.seekg (0, std::ios::end);
  const int applanixLength = applanixBinaryLogFile.tellg();
  applanixBinaryLogFile.seekg (0, std::ios::beg);
  int vnsPacketCounter = 0;
  int vnpPacketCounter = 0;
  int dmiPacketCounter = 0;
  int fwsPacketCounter = 0;
  int rwsPacketCounter = 0;
  int st1PacketCounter = 0;
  while (applanixBinaryLogFile.tellg() != applanixLength) {
    double timestamp;
    applanixLogReader >> timestamp;
    std::shared_ptr<Packet> packet = applanixLogReader.readPacket();
    if (packet->instanceOfGroup()) {
      const Group& group = packet->groupCast();
      if (group.instanceOf<VehicleNavigationSolution>()) {
        const VehicleNavigationSolution& vns =
          group.typeCast<VehicleNavigationSolution>();
        auto vnsMsg = boost::make_shared<poslv::VehicleNavigationSolutionMsg>();
        vnsMsg->header.stamp = ros::Time(timestamp);
        vnsMsg->header.frame_id = "/poslv_link";
        vnsMsg->header.seq = vnsPacketCounter++;
        vnsMsg->timeDistance.time1 = vns.mTimeDistance.mTime1;
        vnsMsg->timeDistance.time2 = vns.mTimeDistance.mTime2;
        vnsMsg->timeDistance.distanceTag = vns.mTimeDistance.mDistanceTag;
        vnsMsg->timeDistance.timeType = vns.mTimeDistance.mTimeType;
        vnsMsg->timeDistance.distanceType = vns.mTimeDistance.mDistanceType;
        vnsMsg->latitude = vns.mLatitude;
        vnsMsg->longitude = vns.mLongitude;
        vnsMsg->altitude = vns.mAltitude;
        vnsMsg->northVelocity = vns.mNorthVelocity;
        vnsMsg->eastVelocity = vns.mEastVelocity;
        vnsMsg->downVelocity = vns.mDownVelocity;
        vnsMsg->roll = vns.mRoll;
        vnsMsg->pitch = vns.mPitch;
        vnsMsg->heading = vns.mHeading;
        vnsMsg->wanderAngle = vns.mWanderAngle;
        vnsMsg->trackAngle = vns.mTrackAngle;
        vnsMsg->speed = vns.mSpeed;
        vnsMsg->angularRateLong = vns.mAngularRateLong;
        vnsMsg->angularRateTrans = vns.mAngularRateTrans;
        vnsMsg->angularRateDown = vns.mAngularRateDown;
        vnsMsg->accLong = vns.mAccLong;
        vnsMsg->accTrans = vns.mAccTrans;
        vnsMsg->accDown = vns.mAccDown;
        vnsMsg->alignementStatus = vns.mAlignementStatus;
        bag.write("/poslv/vehicle_navigation_solution", ros::Time(timestamp),
          vnsMsg);
      }
      if (group.instanceOf<VehicleNavigationPerformance>()) {
        const VehicleNavigationPerformance& vnp =
          group.typeCast<VehicleNavigationPerformance>();
        auto vnpMsg =
          boost::make_shared<poslv::VehicleNavigationPerformanceMsg>();
        vnpMsg->header.stamp = ros::Time(timestamp);
        vnpMsg->header.frame_id = "poslv_link";
        vnpMsg->header.seq = vnpPacketCounter++;
        vnpMsg->timeDistance.time1 = vnp.mTimeDistance.mTime1;
        vnpMsg->timeDistance.time2 = vnp.mTimeDistance.mTime2;
        vnpMsg->timeDistance.distanceTag = vnp.mTimeDistance.mDistanceTag;
        vnpMsg->timeDistance.timeType = vnp.mTimeDistance.mTimeType;
        vnpMsg->timeDistance.distanceType = vnp.mTimeDistance.mDistanceType;
        vnpMsg->northPositionRMSError = vnp.mNorthPositionRMSError;
        vnpMsg->eastPositionRMSError = vnp.mEastPositionRMSError;
        vnpMsg->downPositionRMSError = vnp.mDownPositionRMSError;
        vnpMsg->northVelocityRMSError = vnp.mNorthVelocityRMSError;
        vnpMsg->eastVelocityRMSError = vnp.mEastVelocityRMSError;
        vnpMsg->downVelocityRMSError = vnp.mDownVelocityRMSError;
        vnpMsg->rollRMSError = vnp.mRollRMSError;
        vnpMsg->pitchRMSError = vnp.mPitchRMSError;
        vnpMsg->headingRMSError = vnp.mHeadingRMSError;
        vnpMsg->errorEllipsoidSemiMajor = vnp.mErrorEllipsoidSemiMajor;
        vnpMsg->errorEllipsoidSemiMinor = vnp.mErrorEllipsoidSemiMinor;
        vnpMsg->errorEllipsoidOrientation = vnp.mErrorEllipsoidOrientation;
        bag.write("/poslv/vehicle_navigation_performance", ros::Time(timestamp),
          vnpMsg);
      }
      if (group.instanceOf<TimeTaggedDMIData>()) {
        const TimeTaggedDMIData& dmi =
          group.typeCast<TimeTaggedDMIData>();
        auto dmiMsg = boost::make_shared<poslv::TimeTaggedDMIDataMsg>();
        dmiMsg->header.stamp = ros::Time(timestamp);
        dmiMsg->header.frame_id = "/poslv_link";
        dmiMsg->header.seq = dmiPacketCounter++;
        dmiMsg->timeDistance.time1 = dmi.mTimeDistance.mTime1;
        dmiMsg->timeDistance.time2 = dmi.mTimeDistance.mTime2;
        dmiMsg->timeDistance.distanceTag = dmi.mTimeDistance.mDistanceTag;
        dmiMsg->timeDistance.timeType = dmi.mTimeDistance.mTimeType;
        dmiMsg->timeDistance.distanceType = dmi.mTimeDistance.mDistanceType;
        dmiMsg->signedDistanceTraveled = dmi.mSignedDistanceTraveled;
        dmiMsg->unsignedDistanceTraveled = dmi.mUnsignedDistanceTraveled;
        dmiMsg->dmiScaleFactor = dmi.mDMIScaleFactor;
        dmiMsg->dataStatus = dmi.mDataStatus;
        dmiMsg->dmiType = dmi.mDMIType;
        dmiMsg->dmiDataRate = dmi.mDMIDataRate;
        bag.write("/poslv/time_tagged_dmi_data", ros::Time(timestamp), dmiMsg);
      }
    }
  }

  std::ifstream canBinaryLogFile(argv[2]);
  if (!canBinaryLogFile.is_open())
    throw IOException("File opening failed");
  BinaryStreamReader<std::ifstream> canLogReader(canBinaryLogFile);
  canBinaryLogFile.seekg (0, std::ios::end);
  const int canLength = canBinaryLogFile.tellg();
  canBinaryLogFile.seekg (0, std::ios::beg);
  while (canBinaryLogFile.tellg() != canLength) {
    double timestamp;
    canLogReader >> timestamp;
    int typeID;
    canLogReader >> typeID;
    std::shared_ptr<PRIUSMessage>
      priusMessage(Factory<int, PRIUSMessage>::getInstance().create(
      typeID));
    canLogReader >> *priusMessage;
    if (priusMessage->instanceOf<FrontWheelsSpeed>()) {
      const FrontWheelsSpeed& fws =
        priusMessage->typeCast<FrontWheelsSpeed>();
      auto fwsMsg = boost::make_shared<can_prius::FrontWheelsSpeedMsg>();
      fwsMsg->header.stamp = ros::Time(timestamp);
      fwsMsg->header.frame_id = "/can_link";
      fwsMsg->header.seq = fwsPacketCounter++;
      fwsMsg->Right = fws.mRight;
      fwsMsg->Left = fws.mLeft;
      bag.write("/can_prius/front_wheels_speed", ros::Time(timestamp), fwsMsg);
    }
    else if (priusMessage->instanceOf<RearWheelsSpeed>()) {
      const RearWheelsSpeed& rws =
        priusMessage->typeCast<RearWheelsSpeed>();
      auto rwsMsg = boost::make_shared<can_prius::RearWheelsSpeedMsg>();
      rwsMsg->header.stamp = ros::Time(timestamp);
      rwsMsg->header.frame_id = "/can_link";
      rwsMsg->header.seq = rwsPacketCounter++;
      rwsMsg->Right = rws.mRight;
      rwsMsg->Left = rws.mLeft;
      bag.write("/can_prius/rear_wheels_speed", ros::Time(timestamp), rwsMsg);
    }
    else if (priusMessage->instanceOf<Steering1>()) {
      const Steering1& st = priusMessage->typeCast<Steering1>();
      auto stMsg = boost::make_shared<can_prius::Steering1Msg>();
      stMsg->header.stamp = ros::Time(timestamp);
      stMsg->header.frame_id = "/can_link";
      stMsg->header.seq = st1PacketCounter++;
      stMsg->value = st.mValue;
      bag.write("/can_prius/steering1", ros::Time(timestamp), stMsg);
    }
  }
  return 0;
}
