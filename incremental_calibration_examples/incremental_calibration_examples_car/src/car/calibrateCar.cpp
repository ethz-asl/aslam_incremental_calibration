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
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

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

#include <aslam/splines/OPTBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>
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

  std::cout << "Parsing configuration parameters..." << std::endl;
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[2]);
  const double translationSplineLambda =
    propertyTree.getDouble("calibrator/splines/translationSplineLambda");
  const double rotationSplineLambda =
    propertyTree.getDouble("calibrator/splines/rotationSplineLambda");
  const int splineKnotsPerSecond =
    propertyTree.getInt("calibrator/splines/splineKnotsPerSecond");
  const int translationSplineOrder =
    propertyTree.getInt("calibrator/splines/translationSplineOrder");
  const int rotationSplineOrder =
    propertyTree.getInt("calibrator/splines/rotationSplineOrder");
  const double L =
    propertyTree.getDouble("calibrator/odometry/intrinsic/wheelBase");
  const double e_r =
    propertyTree.getDouble("calibrator/odometry/intrinsic/halfRearTrack");
  const double e_f =
    propertyTree.getDouble("calibrator/odometry/intrinsic/halfFrontTrack");
  const double a0 =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "steeringCoefficient0");
  const double a1 =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "steeringCoefficient1");
  const double a2 =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "steeringCoefficient2");
  const double a3 =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "steeringCoefficient3");
  const double k_rl =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "rearLeftWheelCoefficient");
  const double k_rr =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "rearRightWheelCoefficient");
  const double k_fl =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "frontLeftWheelCoefficient");
  const double k_fr =
    propertyTree.getDouble("calibrator/odometry/intrinsic/"
    "frontRightWheelCoefficient");
  const double k_dmi =
    propertyTree.getDouble("calibrator/odometry/intrinsic/dmiCoefficient");
  const double t_io_x =
    propertyTree.getDouble("calibrator/odometry/extrinsic/translation/x");
  const double t_io_y =
    propertyTree.getDouble("calibrator/odometry/extrinsic/translation/y");
  const double t_io_z =
    propertyTree.getDouble("calibrator/odometry/extrinsic/translation/z");
  const double C_io_yaw =
    propertyTree.getDouble("calibrator/odometry/extrinsic/rotation/yaw");
  const double C_io_pitch =
    propertyTree.getDouble("calibrator/odometry/extrinsic/rotation/pitch");
  const double C_io_roll =
    propertyTree.getDouble("calibrator/odometry/extrinsic/rotation/roll");
  const int hallSensorCutOff =
    propertyTree.getInt("calibrator/odometry/noise/hallSensorCutoff");
  const double rlwPercentError =
    propertyTree.getDouble("calibrator/odometry/noise/rlwPercentError");
  const double rrwPercentError =
    propertyTree.getDouble("calibrator/odometry/noise/rrwPercentError");
  const double flwPercentError =
    propertyTree.getDouble("calibrator/odometry/noise/flwPercentError");
  const double frwPercentError =
    propertyTree.getDouble("calibrator/odometry/noise/frwPercentError");
  const double sigma2_dmi =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_dmi");
  const double dmiPercentError =
    propertyTree.getDouble("calibrator/odometry/noise/dmiPercentError");
  const double sigma2_st =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_st");
  const double minSpeed =
    propertyTree.getDouble("calibrator/odometry/noise/minSpeed");
  const double sigma2_vy =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_vy");
  const double sigma2_vz =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_vz");
  const double sigma2_omx =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_omx");
  const double sigma2_omy =
    propertyTree.getDouble("calibrator/odometry/noise/sigma2_omy");
  const bool colNorm =
    propertyTree.getBool("calibrator/optimizer/linearSolver/colNorm");
  const double qrTol =
    propertyTree.getDouble("calibrator/optimizer/linearSolver/qrTol");
  const double normTol =
    propertyTree.getDouble("calibrator/optimizer/linearSolver/normTol");
  const double epsTol =
    propertyTree.getDouble("calibrator/optimizer/marginalizer/epsTol");
  const double miTol =
    propertyTree.getDouble("calibrator/optimizer/incremental/miTol");
  const double windowDuration =
    propertyTree.getDouble("calibrator/optimizer/incremental/windowDuration");
  const int maxIterations =
    propertyTree.getInt("calibrator/optimizer/maxIterations");
  const bool useDMI =
    propertyTree.getBool("calibrator/odometry/sensors/dmi");
  const bool useFws =
    propertyTree.getBool("calibrator/odometry/sensors/fws");
  const bool useRws =
    propertyTree.getBool("calibrator/odometry/sensors/rws");
  const bool useSt =
    propertyTree.getBool("calibrator/odometry/sensors/st");
  const bool useVm =
    propertyTree.getBool("calibrator/odometry/sensors/vm");

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
  CarCalibrator::CalibrationDesignVariables dv;
  dv.intrinsicOdoDesignVariable =
    boost::make_shared<VectorDesignVariable<12> >(
    (VectorDesignVariable<12>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr, k_dmi).finished());
  dv.intrinsicOdoDesignVariable->setActive(true);
  dv.extrinsicOdoTranslationDesignVariable =
    boost::make_shared<EuclideanPoint>(Eigen::Vector3d(t_io_x, t_io_y, t_io_z));
  dv.extrinsicOdoTranslationDesignVariable->setActive(true);
  auto ypr = boost::make_shared<EulerAnglesYawPitchRoll>();
  dv.extrinsicOdoRotationDesignVariable =
    boost::make_shared<RotationQuaternion>(ypr->parametersToRotationMatrix(
    Eigen::Vector3d(C_io_yaw, C_io_pitch, C_io_roll)));
  dv.extrinsicOdoRotationDesignVariable->setActive(true);
  IncrementalEstimator::Options estimatorOptions;
  estimatorOptions._miTol = miTol;
  estimatorOptions._qrTol = qrTol;
  estimatorOptions._verbose = true;
  estimatorOptions._colNorm = colNorm;
  estimatorOptions._maxIterations = maxIterations;
  estimatorOptions._normTol = normTol;
  estimatorOptions._epsTolSVD = epsTol;
  auto estimator = boost::make_shared<IncrementalEstimator>(1,
    estimatorOptions);
  CarCalibrator::Options calibratorOptions;
  calibratorOptions.transSplineLambda = translationSplineLambda;
  calibratorOptions.rotSplineLambda = rotationSplineLambda;
  calibratorOptions.splineKnotsPerSecond = splineKnotsPerSecond;
  calibratorOptions.transSplineOrder = translationSplineOrder;
  calibratorOptions.rotSplineOrder = rotationSplineOrder;
  calibratorOptions.linearVelocityTolerance = minSpeed;
  calibratorOptions.dmiPercentError = dmiPercentError;
  calibratorOptions.dmiVariance = sigma2_dmi;
  calibratorOptions.flwPercentError = flwPercentError;
  calibratorOptions.frwPercentError = frwPercentError;
  calibratorOptions.rlwPercentError = rlwPercentError;
  calibratorOptions.rrwPercentError = rrwPercentError;
  calibratorOptions.steeringVariance = sigma2_st;
  calibratorOptions.wheelSpeedSensorCutoff = hallSensorCutOff;
  calibratorOptions.vyVariance = sigma2_vy;
  calibratorOptions.vzVariance = sigma2_vz;
  calibratorOptions.omxVariance = sigma2_omx;
  calibratorOptions.omyVariance = sigma2_omy;
  calibratorOptions.useVm = useVm;
  calibratorOptions.verbose = true;
  calibratorOptions.windowDuration = windowDuration;
  CarCalibrator calibrator(estimator, dv, calibratorOptions);
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

  std::cout << std::fixed << std::setprecision(18) << "Covariance: "
    << std::endl
    << estimator->getMarginalizedCovariance().diagonal().transpose()
    << std::endl;

  std::cout << "Outputting spline data after optimization..." << std::endl;
  std::ofstream applanixSplineFile("applanix-spline.txt");
  auto problem = estimator->getProblem();
  size_t numBatches = problem->getNumOptimizationProblems();
  for (size_t i = 0; i < numBatches; ++i) {
    auto batch = problem->getOptimizationProblem(i);
    auto spline = dynamic_cast<const OptimizationProblemSpline*>(batch)
      ->getTranslationSpline();
    for (auto it = spline->begin(); it != spline->end(); ++it) {
      auto translationExpressionFactory =
        spline->getExpressionFactoryAt<0>(it.getTime());
      applanixSplineFile << std::fixed << std::setprecision(18)
        << it.getTime() << " "
        << translationExpressionFactory.getValueExpression().toValue().
        transpose() << std::endl;
    }
  }

  return 0;
}
