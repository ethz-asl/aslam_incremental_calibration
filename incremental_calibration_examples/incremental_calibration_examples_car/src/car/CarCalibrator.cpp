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

#include <boost/make_shared.hpp>

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <bsplines/BSplineFitter.hpp>

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
#include "aslam/calibration/car/OptimizationProblemSpline.h"

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
          !calibrationDesignVariables.intrinsicCANDesignVariable ||
          !calibrationDesignVariables.intrinsicDMIDesignVariable ||
          !calibrationDesignVariables.
            extrinsicOdometryTranslationDesignVariable ||
          !calibrationDesignVariables.extrinsicOdometryRotationDesignVariable)
        throw InvalidOperationException("CarCalibrator::CarCalibrator(): "
          "all pointers must be initialized");
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

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CarCalibrator::addMeasurement(const ApplanixNavigationMeasurement&
        data, double timestamp) {
      addMeasurement(timestamp);
      _applanixNavigationMeasurements.push_back(
        std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(const ApplanixEncoderMeasurement& data,
        double timestamp) {
      addMeasurement(timestamp);
      _applanixEncoderMeasurements.push_back(
        std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(const CANFrontWheelsSpeedMeasurement&
        data, double timestamp) {
      addMeasurement(timestamp);
      _canFrontWheelsSpeedMeasurements.push_back(
        std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(const CANRearWheelsSpeedMeasurement&
        data, double timestamp) {
      addMeasurement(timestamp);
      _canRearWheelsSpeedMeasurements.push_back(
        std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(const CANSteeringMeasurement& data,
        double timestamp) {
      addMeasurement(timestamp);
      _canSteeringMeasurements.push_back(
        std::make_pair(timestamp, data));
    }

    void CarCalibrator::addMeasurement(double timestamp) {
      if (timestamp < _lastTimestamp)
        throw InvalidOperationException("CarCalibrator::addMeasurement(): "
          "timestamps must be monotically increasing");
      if (_currentBatchStartTimestamp == -1)
        _currentBatchStartTimestamp = timestamp;
      if ((timestamp - _currentBatchStartTimestamp) >=
          _options.windowDuration) {
        addMeasurements();
        clearMeasurements();
        _currentBatchStartTimestamp = timestamp;
      }
      _lastTimestamp = timestamp;
    }

    void CarCalibrator::addMeasurements() {
      auto batch = boost::make_shared<OptimizationProblemSpline>();
      batch->addDesignVariable(
        _calibrationDesignVariables.intrinsicCANDesignVariable, 1);
      batch->addDesignVariable(
        _calibrationDesignVariables.intrinsicDMIDesignVariable, 1);
      batch->addDesignVariable(
        _calibrationDesignVariables.extrinsicOdometryTranslationDesignVariable,
        1);
      batch->addDesignVariable(
        _calibrationDesignVariables.extrinsicOdometryRotationDesignVariable,
        1);
      addErrorTerms(_applanixNavigationMeasurements, batch);
      addErrorTerms(_applanixEncoderMeasurements, batch);
      addErrorTerms(_canFrontWheelsSpeedMeasurements, batch);
      addErrorTerms(_canRearWheelsSpeedMeasurements, batch);
      addErrorTerms(_canSteeringMeasurements, batch);
      batch->setGroupsOrdering({0, 1});
      if (_options.verbose) {
        std::cout << "calibration before batch: " << std::endl;
        std::cout << "CAN intrinsic: " << std::endl <<
          *_calibrationDesignVariables.intrinsicCANDesignVariable << std::endl;
        std::cout << "DMI intrinsic: " << std::endl <<
          *_calibrationDesignVariables.intrinsicDMIDesignVariable << std::endl;
        Eigen::MatrixXd t_io;
        _calibrationDesignVariables.extrinsicOdometryTranslationDesignVariable
          ->getParameters(t_io);
        std::cout << "IMU-odometry translation: " << std::endl <<
          t_io.transpose() << std::endl;
        const EulerAnglesYawPitchRoll ypr;
        Eigen::MatrixXd q_io;
        _calibrationDesignVariables.extrinsicOdometryRotationDesignVariable
          ->getParameters(q_io);
        std::cout << "IMU-odometry rotation: " << std::endl <<
          ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;
      }
      IncrementalEstimator::ReturnValue ret =
        _estimator->addBatch(batch);
      if (_options.verbose) {
        std::cout << "rank: " << ret._rank << std::endl;
        std::cout << "QR tol: " << ret._qrTol << std::endl;
        std::cout << "MI: " << ret._mi << std::endl;
        std::cout << "Time [s]: " << ret._elapsedTime << std::endl;
        std::cout << "Cholmod memory [MB]: " <<
          ret._cholmodMemoryUsage / 1024.0 / 1024.0 << std::endl;
        ret._batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
        std::cout << "calibration after batch: " << std::endl;
        std::cout << "CAN intrinsic: " << std::endl <<
          *_calibrationDesignVariables.intrinsicCANDesignVariable << std::endl;
        std::cout << "DMI intrinsic: " << std::endl <<
          *_calibrationDesignVariables.intrinsicDMIDesignVariable << std::endl;
        Eigen::MatrixXd t_io;
        _calibrationDesignVariables.extrinsicOdometryTranslationDesignVariable
          ->getParameters(t_io);
        std::cout << "IMU-odometry translation: " << std::endl <<
          t_io.transpose() << std::endl;
        const EulerAnglesYawPitchRoll ypr;
        Eigen::MatrixXd q_io;
        _calibrationDesignVariables.extrinsicOdometryRotationDesignVariable
          ->getParameters(q_io);
        std::cout << "IMU-odometry rotation: " << std::endl <<
          ypr.rotationMatrixToParameters(quat2r(q_io)).transpose() << std::endl;
      }
    }

    void CarCalibrator::addErrorTerms(const ApplanixNavigationMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      const size_t numMeasurements = measurements.size();
      std::vector<double> timestamps;
      timestamps.reserve(numMeasurements);
      std::vector<Eigen::Vector3d> transPoses;
      transPoses.reserve(numMeasurements);
      std::vector<Eigen::Vector4d> rotPoses;
      rotPoses.reserve(numMeasurements);
      const EulerAnglesYawPitchRoll ypr;
      for (size_t i = 0; i < measurements.size(); ++i) {
        Eigen::Vector4d quat = r2quat(
          ypr.parametersToRotationMatrix(Eigen::Vector3d(
          measurements[i].second.yaw,
          measurements[i].second.pitch,
          measurements[i].second.roll)));
        if (i > 0) {
          const Eigen::Vector4d lastRotPose = rotPoses.back();
          if ((lastRotPose + quat).norm() < (lastRotPose - quat).norm())
            quat = -quat;
        }
        timestamps.push_back(measurements[i].first);
        rotPoses.push_back(quat);
        transPoses.push_back(Eigen::Vector3d(
          measurements[i].second.x,
          measurements[i].second.y,
          measurements[i].second.z));
      }
      _splineStartTime = timestamps[0];
      _splineEndTime = timestamps[numMeasurements - 1];
      const double elapsedTime =
        timestamps[numMeasurements - 1] - timestamps[0];
      const int measPerSec = numMeasurements / elapsedTime;
      int numSegments;
      if (measPerSec > _options.poseMeasPerSecDesired)
        numSegments = _options.poseMeasPerSecDesired * elapsedTime;
      else
        numSegments = numMeasurements;
      _translationSpline = boost::make_shared<TranslationSpline>();
      BSplineFitter<TranslationSpline>::initUniformSpline(
        *_translationSpline, timestamps, transPoses, numSegments,
        _options.poseSplineLambda);
      _rotationSpline = boost::make_shared<RotationSpline>();
      BSplineFitter<RotationSpline>::initUniformSpline(
        *_rotationSpline, timestamps, rotPoses, numSegments,
        _options.poseSplineLambda);
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

    void CarCalibrator::addErrorTerms(const ApplanixEncoderMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      double lastTimestamp = -1;
      double lastDistance = -1;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        if (_splineStartTime > it->first || _splineEndTime < it->first)
          continue;
        if (lastTimestamp != -1) {
//          const Eigen::Matrix<double, 1, 1> meas((Eigen::Matrix<double, 1, 1>()
//            << (it->second.signedDistanceTraveled - lastDistance) /
//            (it->second.gpsTimestamp - lastTimestamp)).finished());
//          auto v = getOdometryVelocities(it->first, _translationSpline,
//            _rotationSpline);
//          auto e_dmi = boost::make_shared<ErrorTermDMI>(v.first, v.second,
//            _calibrationDesignVariables.intrinsicDMIDesignVariable.get(), meas,
//            _options.dmiCovariance);
//          batch->addErrorTerm(e_dmi);
        }
//        lastTimestamp = it->second.gpsTimestamp;
        lastDistance = it->second.signedDistanceTraveled;
      }
    }

    void CarCalibrator::addErrorTerms(const CANFrontWheelsSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        if (_splineStartTime > it->first || _splineEndTime < it->first ||
            it->second.left == 0 || it->second.right == 0)
          continue;
        auto v = getOdometryVelocities(it->first, _translationSpline,
          _rotationSpline);
        auto e_fws = boost::make_shared<ErrorTermFws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          _options.fwsCovariance);
        batch->addErrorTerm(e_fws);
      }
    }

    void CarCalibrator::addErrorTerms(const CANRearWheelsSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        if (_splineStartTime > it->first || _splineEndTime < it->first ||
            it->second.left == 0 || it->second.right == 0)
          continue;
        auto v = getOdometryVelocities(it->first, _translationSpline,
          _rotationSpline);
        auto e_rws = boost::make_shared<ErrorTermRws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          _options.rwsCovariance);
        batch->addErrorTerm(e_rws);
      }
    }

    void CarCalibrator::addErrorTerms(const CANSteeringMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        if (_splineStartTime > it->first || _splineEndTime < it->first)
          continue;
        auto v = getOdometryVelocities(it->first, _translationSpline,
          _rotationSpline);
        if (std::fabs(v.first.toValue()(0)) < _options.linearVelocityTolerance)
          continue;
        Eigen::Matrix<double, 1, 1> meas;
        meas << it->second.value;
        auto e_st = boost::make_shared<ErrorTermSteering>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(), meas,
          _options.steeringCovariance);
        batch->addErrorTerm(e_st);
      }
    }

    std::pair<EuclideanExpression, EuclideanExpression>
        CarCalibrator::getOdometryVelocities(double timestamp,
        const TranslationSplineSP& translationSpline,
        const RotationSplineSP& rotationSpline) const {
      RotationExpression C_io(
        _calibrationDesignVariables.extrinsicOdometryRotationDesignVariable);
      EuclideanExpression t_io(
        _calibrationDesignVariables.extrinsicOdometryTranslationDesignVariable);
      auto translationExpressionFactory =
        translationSpline->getExpressionFactoryAt<1>(timestamp);
      auto rotationExpressionFactory =
        rotationSpline->getExpressionFactoryAt<1>(timestamp);
      auto C_wi = Vector2RotationQuaternionExpressionAdapter::adapt(
        rotationExpressionFactory.getValueExpression());
      auto v_ii = C_wi.inverse() *
        translationExpressionFactory.getValueExpression(1);
      auto om_ii = -(C_wi.inverse() *
        rotationExpressionFactory.getAngularVelocityExpression());
      auto v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
      auto om_oo = C_io.inverse() * om_ii;
      return std::make_pair(v_oo, om_oo);
    }

    void CarCalibrator::clearMeasurements() {
      _applanixNavigationMeasurements.clear();
      _applanixEncoderMeasurements.clear();
      _canFrontWheelsSpeedMeasurements.clear();
      _canRearWheelsSpeedMeasurements.clear();
      _canSteeringMeasurements.clear();
    }

  }
}
