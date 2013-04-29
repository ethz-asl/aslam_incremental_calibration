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

/** \file OdometryTest.cpp
    \brief This file tests the Odometry classes.
  */

#include <gtest/gtest.h>

#include <sm/eigen/gtest.hpp>

#include "aslam/calibration/car/Odometry.h"
#include "aslam/calibration/car/DifferentialOdometry.h"
#include "aslam/calibration/car/AckermanOdometry.h"

using namespace aslam::calibration;

TEST(AslamCalibrationTestSuite, testOdometry) {
  Odometry::VehicleParameters params = {0.285, 0.285, 0.28, 0.28, 1.285, 1.81,
    0, 1, 0, 0, 1, 1, 1, 1};
  Odometry odometry(params);
  DifferentialOdometry diffOdometry(params);
  AckermanOdometry ackOdometry(params);
  ASSERT_EQ(odometry.getVehicleParameters(), params);
  ASSERT_EQ(diffOdometry.getVehicleParameters(), params);
  ASSERT_EQ(ackOdometry.getVehicleParameters(), params);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d::Zero());
  ASSERT_EQ(diffOdometry.getPose(), Eigen::Vector3d::Zero());
  ASSERT_EQ(ackOdometry.getPose(), Eigen::Vector3d::Zero());
  ASSERT_EQ(odometry.getPoseHistory(),
    std::vector<Eigen::Vector3d>({Eigen::Vector3d::Zero()}));
  ASSERT_EQ(diffOdometry.getPoseHistory(),
    std::vector<Eigen::Vector3d>({Eigen::Vector3d::Zero()}));
  ASSERT_EQ(ackOdometry.getPoseHistory(),
    std::vector<Eigen::Vector3d>({Eigen::Vector3d::Zero()}));
  odometry.updateCOGDisplacement(0.5, 0);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(0.5, 0, 0));
  odometry.updateCOGDisplacement(0, M_PI / 4);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(0.5, 0, M_PI / 4));
  odometry.reset();
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d::Zero());
  odometry.updateCOGVelocity(0.1, 0, 10);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 0, 0));
  odometry.updateCOGVelocity(0.0, M_PI / 16, 8);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 0, M_PI / 2));
  odometry.insertPose(Eigen::Vector3d(1, 1, M_PI / 2));
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 1, M_PI / 2));
  odometry.reset(Eigen::Vector3d(1, 2, M_PI / 6));
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 2, M_PI / 6));
  diffOdometry.updateWheelTranslationalVelocities(1.0, 1.0, 1);
  ASSERT_EQ(diffOdometry.getPose(), Eigen::Vector3d(1, 0, 0));
  diffOdometry.reset();
  diffOdometry.updateWheelRotationalVelocities(M_PI / 4, M_PI / 4, 1.0);
  ASSERT_EQ(diffOdometry.getPose(),
    Eigen::Vector3d(M_PI / 4 * 0.285, 0, 0));
  diffOdometry.reset();
  diffOdometry.updateWheelDisplacements(1.0, 1.0);
  ASSERT_EQ(diffOdometry.getPose(), Eigen::Vector3d(1.0, 0, 0));
  ackOdometry.updateWheelTranslationalVelocitiesSteering(1.0, 1.0, 1.0, 1.0, 0,
   1.0);
  sm::eigen::assertNear(ackOdometry.getPose(), Eigen::Vector3d(1.0, 0, 0),
    1e-6, SM_SOURCE_FILE_POS, "Ackerman odometry failed");
  ackOdometry.reset();
  ackOdometry.updateWheelRotationalVelocitiesSteering(M_PI / 4, M_PI / 4,
    M_PI / 4, M_PI / 4, 0, 1.0);
  sm::eigen::assertNear(ackOdometry.getPose(),
    Eigen::Vector3d(M_PI / 4 * 0.285, 0, 0), 1e-2, SM_SOURCE_FILE_POS,
    "Ackerman odometry failed");
  ackOdometry.reset();
  ackOdometry.updateWheelDisplacementsSteering(1.0, 1.0, 1.0, 1.0, 0);
  sm::eigen::assertNear(ackOdometry.getPose(), Eigen::Vector3d(1.0, 0, 0),
    1e-6, SM_SOURCE_FILE_POS, "Ackerman odometry failed");
}
