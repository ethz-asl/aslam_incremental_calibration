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

#include "aslam/calibration/car/CarCalibrator.h"

#include <cmath>
#include <limits>
#include <iostream>
#include <iomanip>

#include <boost/make_shared.hpp>

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <sm/PropertyTree.hpp>

#include <bsplines/BSplineFitter.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/calibration/exceptions/InvalidOperationException.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>
#include <aslam/calibration/core/IncrementalEstimator.h>

#include "aslam/calibration/car/ErrorTermPose.h"
#include "aslam/calibration/car/ErrorTermFws.h"
#include "aslam/calibration/car/ErrorTermRws.h"
#include "aslam/calibration/car/ErrorTermSteering.h"
#include "aslam/calibration/car/ErrorTermDMI.h"
#include "aslam/calibration/car/ErrorTermVehicleModel.h"
#include "aslam/calibration/car/OptimizationProblemSpline.h"
#include "aslam/calibration/car/ApplanixNavigationMeasurement.h"
#include "aslam/calibration/car/WheelsSpeedMeasurement.h"
#include "aslam/calibration/car/SteeringMeasurement.h"
#include "aslam/calibration/car/ApplanixDMIMeasurement.h"
#include "aslam/calibration/car/utils.h"

using namespace sm::timing;
using namespace sm::kinematics;
using namespace bsplines;
using namespace aslam::splines;
using namespace aslam::backend;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CarCalibrator::CarCalibrator(const IncrementalEstimatorSP& estimator, const
        CalibrationDesignVariables& calibrationDesignVariables, const Options&
        options) :
        _options(options),
        _calibrationDesignVariables(calibrationDesignVariables),
        _estimator(estimator),
        _currentBatchStartTimestamp(-1),
        _lastTimestamp(-1) {
      if (!estimator ||
          !calibrationDesignVariables.intrinsicOdoDesignVariable ||
          !calibrationDesignVariables.extrinsicOdoTranslationDesignVariable ||
          !calibrationDesignVariables.extrinsicOdoRotationDesignVariable)
        throw InvalidOperationException("CarCalibrator::CarCalibrator(): "
          "all pointers must be initialized");
    }

    CarCalibrator::CarCalibrator(const sm::PropertyTree& config) :
        _currentBatchStartTimestamp(-1),
        _lastTimestamp(-1) {
      // create the underlying estimator
      _estimator = boost::make_shared<IncrementalEstimator>(
        sm::PropertyTree(config, "estimator"));

      // sets the options for the calibrator
      _options.windowDuration = config.getDouble("windowDuration",
        _options.windowDuration);
      _options.transSplineLambda = config.getDouble("splines/transSplineLambda",
        _options.transSplineLambda);
      _options.rotSplineLambda = config.getDouble("splines/rotSplineLambda",
        _options.rotSplineLambda);
      _options.splineKnotsPerSecond = config.getInt(
        "splines/splineKnotsPerSecond", _options.splineKnotsPerSecond);
      _options.transSplineOrder = config.getInt("splines/transSplineOrder",
        _options.transSplineOrder);
      _options.rotSplineOrder = config.getInt("splines/rotSplineOrder",
        _options.rotSplineOrder);
      _options.linearVelocityTolerance = config.getDouble(
        "odometry/linearVelocityTolerance", _options.linearVelocityTolerance);
      _options.dmiPercentError = config.getDouble(
        "odometry/sensors/dmi/noise/dmiPercentError", _options.dmiPercentError);
      _options.dmiVariance = config.getDouble(
        "odometry/sensors/dmi/noise/dmiVariance", _options.dmiVariance);
      _options.flwPercentError = config.getDouble(
        "odometry/sensors/fws/noise/flwPercentError", _options.flwPercentError);
      _options.flwVariance = config.getDouble(
        "odometry/sensors/fws/noise/flwVariance", _options.flwVariance);
      _options.frwPercentError = config.getDouble(
        "odometry/sensors/fws/noise/frwPercentError", _options.frwPercentError);
      _options.frwVariance = config.getDouble(
        "odometry/sensors/dmi/fws/frwVariance", _options.frwVariance);
      _options.rlwPercentError = config.getDouble(
        "odometry/sensors/rws/noise/rlwPercentError", _options.rlwPercentError);
      _options.rlwVariance = config.getDouble(
        "odometry/sensors/rws/noise/rlwVariance", _options.rlwVariance);
      _options.rrwPercentError = config.getDouble(
        "odometry/sensors/rws/noise/rrwPercentError", _options.rrwPercentError);
      _options.rrwVariance = config.getDouble(
        "odometry/sensors/rws/noise/rrwVariance", _options.rrwVariance);
      _options.steeringVariance = config.getDouble(
        "odometry/sensors/st/noise/steeringVariance",
        _options.steeringVariance);
      _options.wheelSpeedSensorCutoff = config.getInt(
        "odometry/sensors/wheelSpeedSensorCutoff",
        _options.wheelSpeedSensorCutoff);
      _options.vyVariance = config.getDouble(
        "odometry/constraints/noise/vyVariance", _options.vyVariance);
      _options.vzVariance = config.getDouble(
        "odometry/constraints/noise/vzVariance", _options.vzVariance);
      _options.omxVariance = config.getDouble(
        "odometry/constraints/noise/omxVariance", _options.omxVariance);
      _options.omyVariance = config.getDouble(
        "odometry/constraints/noise/omyVariance", _options.omyVariance);
      _options.useVm = config.getBool("odometry/constraints/active",
        _options.useVm);
      _options.verbose = config.getBool("verbose", _options.verbose);

      // create the odometry intrinsic calibration variables
      const double L = config.getDouble("odometry/intrinsics/wheelBase");
      const double e_r = config.getDouble("odometry/intrinsics/halfRearTrack");
      const double e_f = config.getDouble("odometry/intrinsics/halfFrontTrack");
      const double a0 =
        config.getDouble("odometry/intrinsics/steeringCoefficient0");
      const double a1 =
        config.getDouble("odometry/intrinsics/steeringCoefficient1");
      const double a2 =
        config.getDouble("odometry/intrinsics/steeringCoefficient2");
      const double a3 =
        config.getDouble("odometry/intrinsics/steeringCoefficient3");
      const double k_rl =
        config.getDouble("odometry/intrinsics/rlwCoefficient");
      const double k_rr =
        config.getDouble("odometry/intrinsics/rrwCoefficient");
      const double k_fl =
        config.getDouble("odometry/intrinsics/flwCoefficient");
      const double k_fr =
        config.getDouble("odometry/intrinsics/frwCoefficient");
      const double k_dmi =
        config.getDouble("odometry/intrinsics/dmiCoefficient");
      _calibrationDesignVariables.intrinsicOdoDesignVariable =
        boost::make_shared<VectorDesignVariable<12> >(
        (VectorDesignVariable<12>::Container() <<
        L, e_r, e_f, a0, a1, a2, a3, k_rl, k_rr, k_fl, k_fr, k_dmi).finished());
      _calibrationDesignVariables.intrinsicOdoDesignVariable->setActive(true);

      // create the odometry extrinsic calibration variables
      const double t_io_x =
        config.getDouble("odometry/extrinsics/translation/x");
      const double t_io_y =
        config.getDouble("odometry/extrinsics/translation/y");
      const double t_io_z =
        config.getDouble("odometry/extrinsics/translation/z");
      const double C_io_yaw =
        config.getDouble("odometry/extrinsics/rotation/yaw");
      const double C_io_pitch =
        config.getDouble("odometry/extrinsics/rotation/pitch");
      const double C_io_roll =
        config.getDouble("odometry/extrinsics/rotation/roll");
      _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable =
        boost::make_shared<EuclideanPoint>(Eigen::Vector3d(t_io_x, t_io_y,
        t_io_z));
      _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable
        ->setActive(true);
      EulerAnglesYawPitchRoll ypr;
      _calibrationDesignVariables.extrinsicOdoRotationDesignVariable =
        boost::make_shared<RotationQuaternion>(ypr.parametersToRotationMatrix(
        Eigen::Vector3d(C_io_yaw, C_io_pitch, C_io_roll)));
      _calibrationDesignVariables.extrinsicOdoRotationDesignVariable
        ->setActive(true);
    }

    CarCalibrator::~CarCalibrator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const CarCalibrator::Options& CarCalibrator::getOptions() const {
      return _options;
    }

    CarCalibrator::Options& CarCalibrator::getOptions() {
      return _options;
    }

    const CarCalibrator::CalibrationDesignVariables&
        CarCalibrator::getCalibrationDesignVariables() const {
      return _calibrationDesignVariables;
    }

    CarCalibrator::CalibrationDesignVariables&
        CarCalibrator::getCalibrationDesignVariables() {
      return _calibrationDesignVariables;
    }

    const CarCalibrator::IncrementalEstimatorSP CarCalibrator::getEstimator()
        const {
      return _estimator;
    }

    CarCalibrator::IncrementalEstimatorSP CarCalibrator::getEstimator() {
      return _estimator;
    }

    bool CarCalibrator::unprocessedMeasurements() const {
      return !_navigationMeasurements.empty() ||
        !_dmiMeasurements.empty() ||
        !_frontWheelsSpeedMeasurements.empty() ||
        !_rearWheelsSpeedMeasurements.empty() ||
        !_steeringMeasurements.empty();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CarCalibrator::addNavigationMeasurement(const
        ApplanixNavigationMeasurement& data, NsecTime timestamp) {
      addMeasurement(timestamp);
      _navigationMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addDMIMeasurement(const ApplanixDMIMeasurement& data,
        NsecTime timestamp) {
      addMeasurement(timestamp);
      _dmiMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addFrontWheelsMeasurement(const WheelsSpeedMeasurement&
        data, NsecTime timestamp) {
      addMeasurement(timestamp);
      _frontWheelsSpeedMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addRearWheelsMeasurement(const WheelsSpeedMeasurement&
        data, NsecTime timestamp) {
      addMeasurement(timestamp);
      _rearWheelsSpeedMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addSteeringMeasurement(const SteeringMeasurement& data,
        NsecTime timestamp) {
      addMeasurement(timestamp);
      _steeringMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(NsecTime timestamp) {
      _lastTimestamp = timestamp;
      if (_currentBatchStartTimestamp == -1)
        _currentBatchStartTimestamp = timestamp;
      if (nsecToSec(timestamp - _currentBatchStartTimestamp) >=
          _options.windowDuration)
        addMeasurements();
    }

    void CarCalibrator::addMeasurements() {
      auto batch = boost::make_shared<OptimizationProblemSpline>();
      batch->addDesignVariable(
        _calibrationDesignVariables.intrinsicOdoDesignVariable, 1);
      batch->addDesignVariable(
        _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable, 1);
      batch->addDesignVariable(
        _calibrationDesignVariables.extrinsicOdoRotationDesignVariable, 1);
      addNavigationErrorTerms(_navigationMeasurements, batch);
      addDMIErrorTerms(_dmiMeasurements, batch);
      addFrontWheelsErrorTerms(_frontWheelsSpeedMeasurements, batch);
      addRearWheelsErrorTerms(_rearWheelsSpeedMeasurements, batch);
      addSteeringErrorTerms(_steeringMeasurements, batch);
      if (_options.useVm)
        addVehicleErrorTerms(batch);
      clearMeasurements();
      _currentBatchStartTimestamp = _lastTimestamp;
      batch->setGroupsOrdering({0, 1});
      if (_options.verbose) {
        std::cout << "calibration before batch: " << std::endl;
        std::cout << "odometry intrinsics: " << std::endl
//          << std::fixed << std::setprecision(18) <<
        <<
          *_calibrationDesignVariables.intrinsicOdoDesignVariable << std::endl;
        Eigen::MatrixXd t_io;
        _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable
          ->getParameters(t_io);
        std::cout << "IMU-odometry translation: " << std::endl <<
          t_io.transpose() << std::endl;
        const EulerAnglesYawPitchRoll ypr;
        Eigen::MatrixXd q_io;
        _calibrationDesignVariables.extrinsicOdoRotationDesignVariable
          ->getParameters(q_io);
        std::cout << "IMU-odometry rotation: " << std::endl <<
          ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;
      }
      IncrementalEstimator::ReturnValue ret = _estimator->addBatch(batch);
      if (_options.verbose) {
        std::cout << "MI: " << ret.mutualInformation << std::endl;
        ret.batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
        std::cout << std::endl;

        std::cout << "calibration after batch: " << std::endl;
        std::cout << "odometry intrinsics: " << std::endl <<
          *_calibrationDesignVariables.intrinsicOdoDesignVariable << std::endl;
        Eigen::MatrixXd t_io;
        _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable
          ->getParameters(t_io);
        std::cout << "IMU-odometry translation: " << std::endl <<
          t_io.transpose() << std::endl;
        const EulerAnglesYawPitchRoll ypr;
        Eigen::MatrixXd q_io;
        _calibrationDesignVariables.extrinsicOdoRotationDesignVariable
          ->getParameters(q_io);
        std::cout << "IMU-odometry rotation: " << std::endl <<
          ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;
      }
    }

    void CarCalibrator::addNavigationErrorTerms(const
        ApplanixNavigationMeasurements& measurements, const
        OptimizationProblemSplineSP& batch) {
      const size_t numMeasurements = measurements.size();
      std::vector<NsecTime> timestamps;
      timestamps.reserve(numMeasurements);
      std::vector<Eigen::Vector3d> transPoses;
      transPoses.reserve(numMeasurements);
      std::vector<Eigen::Vector4d> rotPoses;
      rotPoses.reserve(numMeasurements);
      const EulerAnglesYawPitchRoll ypr;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        Eigen::Vector4d quat = r2quat(
          ypr.parametersToRotationMatrix(Eigen::Vector3d(it->second.yaw,
          it->second.pitch, it->second.roll)));
        if (!rotPoses.empty()) {
          const Eigen::Vector4d lastRotPose = rotPoses.back();
          quat = bestQuat(lastRotPose, quat);
        }
        timestamps.push_back(timestamp);
        rotPoses.push_back(quat);
        transPoses.push_back(Eigen::Vector3d(it->second.x, it->second.y,
          it->second.z));
      }
      const double elapsedTime = (timestamps.back() - timestamps.front()) /
        (double)NsecTimePolicy::getOne();
      const int measPerSec = std::round(numMeasurements / elapsedTime);
      int numSegments;
      if (measPerSec > _options.splineKnotsPerSecond)
        numSegments = std::ceil(_options.splineKnotsPerSecond * elapsedTime);
      else
        numSegments = numMeasurements;
      _translationSpline = boost::make_shared<TranslationSpline>(
        EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::CONF(
        EuclideanBSpline<Eigen::Dynamic, 3,
        NsecTimePolicy>::CONF::ManifoldConf(3), _options.transSplineOrder));
      BSplineFitter<TranslationSpline>::initUniformSpline(*_translationSpline,
        timestamps, transPoses, numSegments, _options.transSplineLambda);
      _rotationSpline = boost::make_shared<RotationSpline>(
        UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::CONF(
        UnitQuaternionBSpline<Eigen::Dynamic,
        NsecTimePolicy>::CONF::ManifoldConf(), _options.rotSplineOrder));
      BSplineFitter<RotationSpline>::initUniformSpline(*_rotationSpline,
        timestamps, rotPoses, numSegments, _options.rotSplineLambda);
      batch->addSpline(_translationSpline, 0);
      batch->addSpline(_rotationSpline, 0);
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        ErrorTermPose::Input xm;
        xm.head<3>() = Eigen::Vector3d(it->second.x, it->second.y,
          it->second.z);
        xm.tail<3>() = Eigen::Vector3d(it->second.yaw, it->second.pitch,
          it->second.roll);
        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
        Q(0, 0) = it->second.x_sigma2;
        Q(1, 1) = it->second.y_sigma2;
        Q(2, 2) = it->second.z_sigma2;
        Q(3, 3) = it->second.yaw_sigma2;
        Q(4, 4) = it->second.pitch_sigma2;
        Q(5, 5) = it->second.roll_sigma2;
        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<0>(it->first);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<0>(it->first);
        auto e_pose = boost::make_shared<ErrorTermPose>(
          TransformationExpression(
          Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression()),
          translationExpressionFactory.getValueExpression()),
          xm, Q);
        batch->addErrorTerm(e_pose);
      }
    }

    void CarCalibrator::addDMIErrorTerms(const ApplanixDMIMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      NsecTime lastTimestamp = -1;
      double lastDistance = -1;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;
        if (lastTimestamp != -1) {
          auto v = getOdometryVelocities(timestamp, _translationSpline,
            _rotationSpline);
          if (std::fabs(v.first.toValue()(0)) <
              _options.linearVelocityTolerance) {
            lastTimestamp = timestamp;
            lastDistance = it->second.signedDistanceTraveled;
            continue;
          }
          const double displacement = it->second.signedDistanceTraveled -
            lastDistance;
          const Eigen::Matrix<double, 1, 1> meas((Eigen::Matrix<double, 1, 1>()
            << displacement / (timestamp - lastTimestamp) *
            (double)NsecTimePolicy::getOne()).finished());
          auto e_dmi = boost::make_shared<ErrorTermDMI>(v.first, v.second,
            _calibrationDesignVariables.intrinsicOdoDesignVariable.get(), meas,
//            (Eigen::Matrix<double, 1, 1>() << _options.dmiPercentError * meas(0)
//            * _options.dmiPercentError * meas(0)).finished());
            (Eigen::Matrix<double, 1, 1>() << _options.dmiVariance).finished());
          batch->addErrorTerm(e_dmi);
        }
        lastTimestamp = timestamp;
        lastDistance = it->second.signedDistanceTraveled;
      }
    }

    void CarCalibrator::addFrontWheelsErrorTerms(const WheelsSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp ||
            it->second.left < _options.wheelSpeedSensorCutoff ||
            it->second.right < _options.wheelSpeedSensorCutoff)
          continue;
        auto v = getOdometryVelocities(timestamp, _translationSpline,
          _rotationSpline);
        if (v.first.toValue()(0) < _options.linearVelocityTolerance)
          continue;
        const double v_oo_x = v.first.toValue()(0);
        const double om_oo_z = v.second.toValue()(2);
        const double L =
          _calibrationDesignVariables.intrinsicOdoDesignVariable->getValue()(0);
        const double e_f =
          _calibrationDesignVariables.intrinsicOdoDesignVariable->getValue()(2);
        const double phi_L = atan(L * om_oo_z / (v_oo_x - e_f * om_oo_z));
        const double phi_R = atan(L * om_oo_z / (v_oo_x + e_f * om_oo_z));
        if (fabs(cos(phi_L)) < std::numeric_limits<double>::epsilon() ||
            fabs(cos(phi_R)) < std::numeric_limits<double>::epsilon())
          continue;
        auto e_fws = boost::make_shared<ErrorTermFws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicOdoDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          (Eigen::Matrix2d() << (_options.flwPercentError * it->second.left) *
            (_options.flwPercentError * it->second.left), 0, 0,
            (_options.frwPercentError * it->second.right) *
            (_options.frwPercentError * it->second.right)).finished());
        batch->addErrorTerm(e_fws);
      }
    }

    void CarCalibrator::addRearWheelsErrorTerms(const WheelsSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp ||
            it->second.left < _options.wheelSpeedSensorCutoff ||
            it->second.right < _options.wheelSpeedSensorCutoff)
          continue;
        auto v = getOdometryVelocities(timestamp, _translationSpline,
          _rotationSpline);
        if (v.first.toValue()(0) < _options.linearVelocityTolerance)
          continue;
        auto e_rws = boost::make_shared<ErrorTermRws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicOdoDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          (Eigen::Matrix2d() << (_options.rlwPercentError * it->second.left) *
            (_options.rlwPercentError * it->second.left), 0, 0,
            (_options.rrwPercentError * it->second.right) *
            (_options.rrwPercentError * it->second.right)).finished());
        batch->addErrorTerm(e_rws);
      }
    }

    void CarCalibrator::addSteeringErrorTerms(const SteeringMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;
        auto v = getOdometryVelocities(timestamp, _translationSpline,
          _rotationSpline);
        if (std::fabs(v.first.toValue()(0)) < _options.linearVelocityTolerance)
          continue;
        Eigen::Matrix<double, 1, 1> meas((Eigen::Matrix<double, 1, 1>() <<
          it->second.value).finished());
        auto e_st = boost::make_shared<ErrorTermSteering>(v.first, v.second,
          _calibrationDesignVariables.intrinsicOdoDesignVariable.get(), meas,
          (Eigen::Matrix<double, 1, 1>() <<
          _options.steeringVariance).finished());
        batch->addErrorTerm(e_st);
      }
    }

    void CarCalibrator::addVehicleErrorTerms(const OptimizationProblemSplineSP&
        batch) {
      for (auto it = _translationSpline->begin();
          it != _translationSpline->end(); ++it) {
        auto timestamp = it.getTime();
        auto v = getOdometryVelocities(timestamp, _translationSpline,
          _rotationSpline);
        if (std::fabs(v.first.toValue()(0)) < _options.linearVelocityTolerance)
          continue;
        ErrorTermVehicleModel::Covariance Q(
          ErrorTermVehicleModel::Covariance::Zero());
        Q(0, 0) = _options.vyVariance; Q(1, 1) = _options.vzVariance;
        Q(2, 2) = _options.omxVariance; Q(3, 3) = _options.omyVariance;
        auto e_vm = boost::make_shared<ErrorTermVehicleModel>(v.first,
          v.second, Q);
        batch->addErrorTerm(e_vm);
      }
    }

    std::pair<EuclideanExpression, EuclideanExpression>
        CarCalibrator::getOdometryVelocities(NsecTime timestamp,
        const TranslationSplineSP& translationSpline,
        const RotationSplineSP& rotationSpline) const {
      // Rotation transforming vectors in odometry frame to IMU frame
      RotationExpression C_io(
        _calibrationDesignVariables.extrinsicOdoRotationDesignVariable);
      // Translation from odometry frame to IMU frame
      EuclideanExpression t_io(
        _calibrationDesignVariables.extrinsicOdoTranslationDesignVariable);
      auto translationExpressionFactory =
        translationSpline->getExpressionFactoryAt<1>(timestamp);
      auto rotationExpressionFactory =
        rotationSpline->getExpressionFactoryAt<1>(timestamp);
      // Rotation transforming vectors in IMU frame to world frame
      auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
        rotationExpressionFactory.getValueExpression());
      // linear velocity of IMU frame w.r. to world frame in IMU frame
      auto v_ii = C_wi.inverse() *
        translationExpressionFactory.getValueExpression(1);
      // angular velocity of IMU frame w.r. to world frame in IMU frame
      auto om_ii = -(C_wi.inverse() *
        rotationExpressionFactory.getAngularVelocityExpression());
      // linear velocity of odometry frame w.r. to world frame in odometry frame
      auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
      // ang. velocity of odometry frame w.r. to world frame in odometry frame
      auto om_oo = C_io.inverse() * om_ii;
      return std::make_pair(v_oo, om_oo);
    }

    void CarCalibrator::clearMeasurements() {
      _navigationMeasurements.clear();
      _dmiMeasurements.clear();
      _frontWheelsSpeedMeasurements.clear();
      _rearWheelsSpeedMeasurements.clear();
      _steeringMeasurements.clear();
    }

  }
}
