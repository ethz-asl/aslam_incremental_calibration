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

#include <Eigen/Core>

#include <boost/make_shared.hpp>

#include <sm/BoostPropertyTree.hpp>

#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

#include <aslam/TrajectoryUtilities.hpp>
#include <aslam/DiscreteTrajectory.hpp>
#include <aslam/SplineTrajectory.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>

#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/core/IncrementalEstimator.h>
//#include <aslam/calibration/core/IncrementalOptimizationProblem.h>

#include "aslam/calibration/car/MeasurementsContainer.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/utils.h"
#include "aslam/calibration/car/CarCalibrator.h"


//#include <aslam/splines/OPTBSpline.hpp>

//#include <bsplines/EuclideanBSpline.hpp>
//#include <bsplines/NsecTimePolicy.hpp>


//#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
//#include "aslam/calibration/car/SteeringMeasurement.h"
//#include "aslam/calibration/car/ApplanixDMIMeasurement.h"
//#include "aslam/calibration/car/OptimizationProblemSpline.h"

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

  std::cout << "Parsing configuration parameters..." << std::endl;
  BoostPropertyTree propertyTree;
  propertyTree.loadXml(argv[1]);
  const size_t numSimulationSteps =
    propertyTree.getInt("calibrator/simulation/numSteps");
  const double simulationStepX =
    propertyTree.getDouble("calibrator/simulation/stepX");
  const double simulationStepY =
    propertyTree.getDouble("calibrator/simulation/stepY");
  const double simulationStepZ =
    propertyTree.getDouble("calibrator/simulation/stepZ");
  const double simulationStepYaw =
    propertyTree.getDouble("calibrator/simulation/stepYaw");
  const double simulationStepPitch =
    propertyTree.getDouble("calibrator/simulation/stepPitch");
  const double simulationStepRoll =
    propertyTree.getDouble("calibrator/simulation/stepRoll");
  const double simulationStepSigmaX =
    propertyTree.getDouble("calibrator/simulation/stepSigmaX");
  const double simulationStepSigmaY =
    propertyTree.getDouble("calibrator/simulation/stepSigmaY");
  const double simulationStepSigmaZ =
    propertyTree.getDouble("calibrator/simulation/stepSigmaZ");
  const double simulationStepSigmaYaw =
    propertyTree.getDouble("calibrator/simulation/stepSigmaYaw");
  const double simulationStepSigmaPitch =
    propertyTree.getDouble("calibrator/simulation/stepSigmaPitch");
  const double simulationStepSigmaRoll =
    propertyTree.getDouble("calibrator/simulation/stepSigmaRoll");
  const size_t simulationStepTime =
    propertyTree.getInt("calibrator/simulation/stepTime");
  const size_t simulationSplineKnotsPerSecond =
    propertyTree.getInt("calibrator/simulation/splineKnotsPerSecond");
  const double simulationSplineLambda =
    propertyTree.getDouble("calibrator/simulation/splineLambda");
  const double applanixFrequency =
    propertyTree.getDouble("calibrator/simulation/applanixFrequency");
  const double wheelSpeedFrequency =
    propertyTree.getDouble("calibrator/simulation/wheelSpeedFrequency");
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

  std::cout << "Generating trajectory..." << std::endl;
  auto ypr = boost::make_shared<EulerAnglesYawPitchRoll>();
  const Transformation deterministicStep(r2quat(ypr->parametersToRotationMatrix(
    Eigen::Vector3d(simulationStepYaw, simulationStepPitch, simulationStepRoll))
    ), Eigen::Vector3d(simulationStepX, simulationStepY, simulationStepZ));
  const Eigen::Vector3d translationStdDev(simulationStepSigmaX,
    simulationStepSigmaY, simulationStepSigmaZ);
  const Eigen::Vector3d yawPitchRollStdDev(simulationStepSigmaYaw,
    simulationStepSigmaPitch, simulationStepSigmaRoll);
  DiscreteTrajectory discreteTrajectory = createTrajectoryRandomWalk(0,
    Transformation(), numSimulationSteps, deterministicStep, translationStdDev,
    yawPitchRollStdDev, simulationStepTime);
  SplineTrajectory splineTrajectory;
  splineTrajectory.initialize(discreteTrajectory,
    simulationSplineKnotsPerSecond, simulationSplineLambda);

  std::cout << "Simulating measurements..." << std::endl;
  MeasurementsContainer<ApplanixNavigationMeasurement>::Type
    navigationMeasurements;
  const Transformation T_io(r2quat(ypr->parametersToRotationMatrix(
    Eigen::Vector3d(C_io_yaw, C_io_pitch, C_io_roll))),
    Eigen::Vector3d(t_io_x, t_io_y, t_io_z));
  simulateNavigationMeasurements(splineTrajectory, applanixFrequency, T_io,
    navigationMeasurements);
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueRearWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    rearWheelsSpeedMeasurements;
  simulateRearWheelsSpeedMeasurements(splineTrajectory, wheelSpeedFrequency,
    rlwPercentError, rrwPercentError, e_r, k_rl, k_rr,
    trueRearWheelsSpeedMeasurements, rearWheelsSpeedMeasurements);
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    trueFrontWheelsSpeedMeasurements;
  MeasurementsContainer<WheelsSpeedMeasurement>::Type
    frontWheelsSpeedMeasurements;
  simulateFrontWheelsSpeedMeasurements(splineTrajectory, wheelSpeedFrequency,
    flwPercentError, frwPercentError, e_f, L, k_fl, k_fr,
    trueFrontWheelsSpeedMeasurements, frontWheelsSpeedMeasurements);

  std::cout << "Optimizing..." << std::endl;
  CarCalibrator::CalibrationDesignVariables dv;
  dv.intrinsicOdoDesignVariable =
    boost::make_shared<VectorDesignVariable<12> >(
    (VectorDesignVariable<12>::Container() <<
    L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr, k_dmi).finished());
  dv.intrinsicOdoDesignVariable->setActive(true);
  dv.extrinsicOdoTranslationDesignVariable =
    boost::make_shared<EuclideanPoint>(Eigen::Vector3d(t_io_x, t_io_y, t_io_z));
  dv.extrinsicOdoTranslationDesignVariable->setActive(true);
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
  for (auto it = navigationMeasurements.cbegin();
      it != navigationMeasurements.cend(); ++it)
    calibrator.addNavigationMeasurement(it->second, it->first);
  for (auto it = rearWheelsSpeedMeasurements.cbegin();
      it != rearWheelsSpeedMeasurements.cend(); ++it)
    calibrator.addRearWheelsMeasurement(it->second, it->first);
  for (auto it = frontWheelsSpeedMeasurements.cbegin();
      it != frontWheelsSpeedMeasurements.cend(); ++it)
    calibrator.addFrontWheelsMeasurement(it->second, it->first);

  if (calibrator.unprocessedMeasurements())
    calibrator.addMeasurements();

  std::cout << "final calibration: " << std::endl;
  std::cout << "Odometry intrinsic: " << std::endl
    << std::fixed << std::setprecision(18) <<
    *dv.intrinsicOdoDesignVariable << std::endl;
  Eigen::MatrixXd t_io;
  dv.extrinsicOdoTranslationDesignVariable->getParameters(t_io);
  std::cout << "IMU-odometry translation: " << std::endl <<
    t_io.transpose() << std::endl;
  Eigen::MatrixXd q_io;
  dv.extrinsicOdoRotationDesignVariable->getParameters(q_io);
  std::cout << "IMU-odometry rotation: " << std::endl <<
    ypr->rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;

  std::cout << std::fixed << std::setprecision(18) << "Covariance: "
    << std::endl
    << estimator->getMarginalizedCovariance().diagonal().transpose()
    << std::endl;

  std::cout << std::fixed << std::setprecision(18) << "Null-space: "
    << std::endl
    << estimator->getMarginalizedNullSpace() << std::endl;

  return 0;
}
