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

/** \file calibrateCarSimulation.cpp
    \brief This file calibrates the car parameters from a simulation.
  */

#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include <sm/BoostPropertyTree.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/TrajectoryUtilities.hpp>
#include <aslam/DiscreteTrajectory.hpp>
#include <aslam/SplineTrajectory.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/core/IncrementalEstimator.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/CarCalibrator.h"

using namespace aslam;
using namespace aslam::calibration;
using namespace sm;
using namespace sm::kinematics;
using namespace sm::timing;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <conf_file>" << std::endl;
    return -1;
  }

  std::cout << "Loading configuration parameters..." << std::endl;
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[1]);

  std::cout << "Generating trajectory..." << std::endl;
  EulerAnglesYawPitchRoll ypr;
  DiscreteTrajectory discreteTrajectory;
  Transformation T_km1(r2quat(ypr.parametersToRotationMatrix(
    Eigen::Vector3d(propertyTree.getDouble("car/simulation/x0/yaw"),
    propertyTree.getDouble("car/simulation/x0/pitch"),
    propertyTree.getDouble("car/simulation/x0/roll")))),
    Eigen::Vector3d(propertyTree.getDouble("car/simulation/x0/x"),
    propertyTree.getDouble("car/simulation/x0/y"),
    propertyTree.getDouble("car/simulation/x0/z")));
  double yaw = M_PI / 32;
  double pitch = 0;
  const Trajectory::NsecTime timestep =
    propertyTree.getInt("car/simulation/timestep");
  const size_t numSteps = propertyTree.getInt("car/simulation/numSteps");
  Trajectory::NsecTime t = 0;
  for (size_t i = 0; i < numSteps; ++i) {
    auto C_km1 = T_km1.C();
    auto t_km1 = T_km1.t();
    Eigen::Vector3d v(10, 0, 0);
    Eigen::Vector3d t_k = t_km1 + C_km1 * v *
      nsecToSec(timestep);
    if (t % 10000000000 == 0) {
      yaw = -yaw;
    }
    if (t % 50000000000)
      pitch = M_PI / 32;
    Eigen::Vector3d om(0, pitch, yaw);
    Eigen::Vector3d incOm = (om * nsecToSec(timestep));
    const double normIncOm = incOm.norm();
    Eigen::Matrix3d C_k;
    if (normIncOm != 0) {
      Eigen::Vector3d incOmNorm = incOm / normIncOm;
      Eigen::Matrix3d incOmCross;
      incOmCross << 0, -incOmNorm(2), incOmNorm(1), incOmNorm(2), 0,
        -incOmNorm(0), -incOmNorm(1), incOmNorm(0), 0;
      C_k = C_km1 * (cos(normIncOm) * Eigen::Matrix3d::Identity()
        + (1 - cos(normIncOm)) * incOmNorm * incOmNorm.transpose() -
        sin(normIncOm) * incOmCross).transpose();
    }
    else
      C_k = C_km1;
    Transformation T_k(r2quat(C_k), t_k);
    discreteTrajectory.addPose(t, T_k);
    t += timestep;
    T_km1 = T_k;
  }
  discreteTrajectory.saveCsvAtSupportTimes("trajectory.csv");
  SplineTrajectory splineTrajectory;
  splineTrajectory.initialize(discreteTrajectory,
    propertyTree.getInt("car/simulation/splineKnotsPerSecond"),
    propertyTree.getDouble("car/simulation/splineLambda"));

  std::cout << "Simulating measurements..." << std::endl;
  MeasurementsContainer<ApplanixNavigationMeasurement>::Type
    navigationMeasurements;
  const double t_io_x = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/translation/x");
  const double t_io_y = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/translation/y");
  const double t_io_z = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/translation/z");
  const double C_io_yaw = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/rotation/yaw");
  const double C_io_pitch = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/rotation/pitch");
  const double C_io_roll = propertyTree.getDouble(
    "car/simulation/calibration/odometry/extrinsics/rotation/roll");
  const Transformation T_io(r2quat(ypr.parametersToRotationMatrix(
    Eigen::Vector3d(C_io_yaw, C_io_pitch, C_io_roll))),
    Eigen::Vector3d(t_io_x, t_io_y, t_io_z));
  simulateNavigationMeasurements(splineTrajectory,
    propertyTree.getDouble("car/simulation/applanixFrequency"), T_io,
    navigationMeasurements);

  std::cout << "Outputting raw Applanix data..." << std::endl;
  std::ofstream applanixRawFile("applanix-raw.txt");
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it)
    applanixRawFile << std::fixed << std::setprecision(18)
      << it->first << " "
      << it->second.x << " " << it->second.y << " " << it->second.z << " "
      << it->second.yaw << " " << it->second.pitch << " "
      << it->second.roll << " "
      << it->second.v_x << " " << it->second.v_y << " " << it->second.v_z << " "
      << it->second.om_x << " " << it->second.om_y << " "
      << it->second.om_z << " "
      << it->second.a_x << " " << it->second.a_y << " " << it->second.a_z << " "
      << std::endl;

  const double L = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/wheelBase");
  const double e_r = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/halfRearTrack");
  const double e_f = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/halfFrontTrack");
  const double k_rl = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/rlwCoefficient");
  const double k_rr = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/rrwCoefficient");
  const double k_fl = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/flwCoefficient");
  const double k_fr = propertyTree.getDouble(
    "car/simulation/calibration/odometry/intrinsics/frwCoefficient");
  const double rlwPercentError = propertyTree.getDouble(
    "car/calibrator/odometry/sensors/rws/noise/rlwPercentError");
  const double rrwPercentError = propertyTree.getDouble(
    "car/calibrator/odometry/sensors/rws/noise/rrwPercentError");
  const double flwPercentError = propertyTree.getDouble(
    "car/calibrator/odometry/sensors/fws/noise/flwPercentError");
  const double frwPercentError = propertyTree.getDouble(
    "car/calibrator/odometry/sensors/fws/noise/frwPercentError");
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueRearWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    rearWheelsSpeedMeasurements;
  simulateRearWheelsSpeedMeasurements(splineTrajectory,
    propertyTree.getDouble("car/simulation/wheelSpeedFrequency"),
    rlwPercentError, rrwPercentError, e_r, k_rl, k_rr,
    trueRearWheelsSpeedMeasurements, rearWheelsSpeedMeasurements);
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueFrontWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    frontWheelsSpeedMeasurements;
  simulateFrontWheelsSpeedMeasurements(splineTrajectory,
    propertyTree.getDouble("car/simulation/wheelSpeedFrequency"),
    flwPercentError, frwPercentError, e_f, L, k_fl, k_fr,
    trueFrontWheelsSpeedMeasurements, frontWheelsSpeedMeasurements);

  std::cout << "Optimizing..." << std::endl;
  CarCalibrator calibrator(PropertyTree(propertyTree, "car/calibrator"));
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it)
    calibrator.addNavigationMeasurement(it->second, it->first);
  for (auto it = trueRearWheelsSpeedMeasurements.cbegin();
      it != trueRearWheelsSpeedMeasurements.cend(); ++it)
    calibrator.addRearWheelsMeasurement(it->second, it->first);
  for (auto it = trueFrontWheelsSpeedMeasurements.cbegin();
      it != trueFrontWheelsSpeedMeasurements.cend(); ++it)
    calibrator.addFrontWheelsMeasurement(it->second, it->first);

  if (calibrator.unprocessedMeasurements())
    calibrator.addMeasurements();

  std::cout << "final calibration: " << std::endl;
  auto dv = calibrator.getCalibrationDesignVariables();
  std::cout << "Odometry intrinsic: " << std::endl << std::fixed
    << std::setprecision(18) << *dv.intrinsicOdoDesignVariable << std::endl;
  Eigen::MatrixXd t_io;
  dv.extrinsicOdoTranslationDesignVariable->getParameters(t_io);
  std::cout << "IMU-odometry translation: " << std::endl <<
    t_io.transpose() << std::endl;
  Eigen::MatrixXd q_io;
  dv.extrinsicOdoRotationDesignVariable->getParameters(q_io);
  std::cout << "IMU-odometry rotation: " << std::endl <<
    ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;

  std::cout << "Covariance: " << std::fixed << std::setprecision(18)
    << std::endl << calibrator.getEstimator()
      ->getMarginalizedCovariance().diagonal().transpose() << std::endl;

  std::cout << std::fixed << std::setprecision(18) << "Null-space: "
    << std::endl
    << calibrator.getEstimator()->getMarginalizedNullSpace() << std::endl;

  return 0;
}
