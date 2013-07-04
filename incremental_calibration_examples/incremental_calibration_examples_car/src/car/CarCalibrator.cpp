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

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <bsplines/BSplinePose.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>

#include <aslam/calibration/exceptions/InvalidOperationException.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/utils.h"
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
      IncrementalEstimator::ReturnValue ret =
        _estimator->addBatch(batch);
    }

    void CarCalibrator::addErrorTerms(const ApplanixNavigationMeasurements&
        measurements, IncrementalEstimator::BatchSP batch) {
      auto rv = boost::make_shared<RotationVector>();
      BSplinePose bspline(_options.poseSplineOrder, rv);
      const size_t numMeasurements = measurements.size();
      Eigen::VectorXd timestamps(numMeasurements);
      Eigen::Matrix<double, 6, Eigen::Dynamic> poses(6, numMeasurements);
      const EulerAnglesYawPitchRoll ypr;
      for (size_t i = 0; i < measurements.size(); ++i) {
        Eigen::Vector3d crv = rv->rotationMatrixToParameters(
          ypr.parametersToRotationMatrix(Eigen::Vector3d(
          measurements[i].second.yaw, measurements[i].second.pitch,
          measurements[i].second.roll)));
        if (i > 0) {
          Eigen::Matrix<double, 6, 1> lastPose = poses.col(i - 1);
          crv = rotVectorNoFlipping(lastPose.tail<3>(), crv);
        }
        timestamps(i) = measurements[i].first;
        Eigen::Matrix<double, 6, 1> pose;
        pose << measurements[i].second.x, measurements[i].second.y,
          measurements[i].second.z, crv;
        poses.col(i) = pose;
      }
      const double elapsedTime =
        timestamps(numMeasurements - 1) - timestamps(0);
      const int measPerSec = numMeasurements / elapsedTime;
      int numSegments;
      if (measPerSec > _options.poseMeasPerSecDesired)
        numSegments = _options.poseMeasPerSecDesired * elapsedTime;
      else
        numSegments = numMeasurements;
      bspline.initPoseSplineSparse(timestamps, poses, numSegments,
        _options.poseSplineLambda);
      _bspdv = boost::make_shared<BSplinePoseDesignVariable>(bspline);
      for (size_t i = 0; i < _bspdv->numDesignVariables(); ++i)
        _bspdv->designVariable(i)->setActive(true);
      dynamic_cast<OptimizationProblemSpline*>(batch.get())->addDesignVariable(
        _bspdv, 0);
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
        auto e_pose = boost::make_shared<ErrorTermPose>(
          _bspdv->transformation(it->first), xm, Q);
        batch->addErrorTerm(e_pose);
      }
    }

    void CarCalibrator::addErrorTerms(const ApplanixEncoderMeasurements&
        measurements, IncrementalEstimator::BatchSP batch) {
      Eigen::Matrix<double, 1, 1> R_dmi;
      R_dmi << _options.dmiVariance;
      double lastTimestamp = -1;
      double lastDistance = -1;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        Eigen::Matrix<double, 1, 1> meas;
        if (lastTimestamp != -1) {
          meas << (it->second.signedDistanceTraveled - lastDistance) /
            (it->first - lastTimestamp);
          auto v = getOdometryVelocities(it->first, _bspdv);
          auto e_dmi = boost::make_shared<ErrorTermDMI>(v.first, v.second,
            _calibrationDesignVariables.intrinsicDMIDesignVariable.get(), meas,
            R_dmi);
          batch->addErrorTerm(e_dmi);
        }
        lastTimestamp = it->first;
        lastDistance = it->second.signedDistanceTraveled;
      }
    }

    void CarCalibrator::addErrorTerms(const CANFrontWheelsSpeedMeasurements&
        measurements, IncrementalEstimator::BatchSP batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto v = getOdometryVelocities(it->first, _bspdv);
        if (it->second.left == 0 || it->second.right == 0 ||
            std::fabs(v.first.toValue()(0)) < _options.linearVelocityTolerance)
          continue;
        auto e_fws = boost::make_shared<ErrorTermFws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          _options.fwsCovariance);
        batch->addErrorTerm(e_fws);
      }
    }

    void CarCalibrator::addErrorTerms(const CANRearWheelsSpeedMeasurements&
        measurements, IncrementalEstimator::BatchSP batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        if (it->second.left == 0 || it->second.right == 0)
          continue;
        auto v = getOdometryVelocities(it->first, _bspdv);
        auto e_rws = boost::make_shared<ErrorTermRws>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(),
          Eigen::Vector2d(it->second.left, it->second.right),
          _options.rwsCovariance);
        batch->addErrorTerm(e_rws);
      }
    }

    void CarCalibrator::addErrorTerms(const CANSteeringMeasurements&
        measurements, IncrementalEstimator::BatchSP batch) {
      Eigen::Matrix<double, 1, 1> R_st;
      R_st << _options.steeringVariance;
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto v = getOdometryVelocities(it->first, _bspdv);
        if (std::fabs(v.first.toValue()(0)) < _options.linearVelocityTolerance)
          continue;
        Eigen::Matrix<double, 1, 1> meas;
        meas << it->second.value;
        auto e_st = boost::make_shared<ErrorTermSteering>(v.first, v.second,
          _calibrationDesignVariables.intrinsicCANDesignVariable.get(), meas,
          R_st);
        batch->addErrorTerm(e_st);
      }
    }

    std::pair<EuclideanExpression, EuclideanExpression>
        CarCalibrator::getOdometryVelocities(double timestamp,
        const BSplinePoseDesignVariableSP& bspdv) const {
      RotationExpression C_io(
        _calibrationDesignVariables.extrinsicOdometryRotationDesignVariable);
      EuclideanExpression t_io(
        _calibrationDesignVariables.extrinsicOdometryTranslationDesignVariable);
      EuclideanExpression v_iw = bspdv->linearVelocity(timestamp);
      RotationExpression C_wi = bspdv->orientation(timestamp);
      EuclideanExpression v_ii = C_wi.inverse() * v_iw;
      EuclideanExpression om_ii = bspdv->angularVelocityBodyFrame(timestamp);
      EuclideanExpression v_oo = C_io.inverse() * (v_ii + om_ii.cross(t_io));
      EuclideanExpression om_oo = C_io.inverse() * om_ii;
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
