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

#include "aslam/calibration/car/algo/CarCalibrator.h"

#include <vector>
#include <cmath>

#include <boost/make_shared.hpp>

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <sm/PropertyTree.hpp>

#include <bsplines/BSplineFitter.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/data-structures/VectorDesignVariable.h>

#include "aslam/calibration/car/error-terms/ErrorTermPose.h"
#include "aslam/calibration/car/error-terms/ErrorTermVelocities.h"
#include "aslam/calibration/car/error-terms/ErrorTermWheel.h"
#include "aslam/calibration/car/error-terms/ErrorTermSteering.h"
#include "aslam/calibration/car/design-variables/OdometryDesignVariables.h"
#include "aslam/calibration/car/algo/OptimizationProblemSpline.h"
#include "aslam/calibration/car/algo/bestQuat.h"
#include "aslam/calibration/car/data/PoseMeasurement.h"
#include "aslam/calibration/car/data/VelocitiesMeasurement.h"
#include "aslam/calibration/car/data/WheelSpeedsMeasurement.h"
#include "aslam/calibration/car/data/SteeringMeasurement.h"
#include "aslam/calibration/car/data/DMIMeasurement.h"

using namespace sm;
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

    CarCalibrator::CarCalibrator(const PropertyTree& config) :
        _currentBatchStartTimestamp(-1),
        _lastTimestamp(-1) {
      // create the underlying estimator
      _estimator = boost::make_shared<IncrementalEstimator>(
        sm::PropertyTree(config, "estimator"));

      // sets the options for the calibrator
      _options = Options(config);

      // create the odometry design variables
      _odometryDesignVariables = boost::make_shared<OdometryDesignVariables>(
        sm::PropertyTree(config, "odometry"));

      // save initial guess
      _odometryVariablesHistory.push_back(
        _odometryDesignVariables->getParameters());
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

    const CarCalibrator::OdometryDesignVariablesSP&
        CarCalibrator::getOdometryDesignVariables() const {
      return _odometryDesignVariables;
    }

    CarCalibrator::OdometryDesignVariablesSP&
        CarCalibrator::getOdometryDesignVariables() {
      return _odometryDesignVariables;
    }

    const CarCalibrator::IncrementalEstimatorSP CarCalibrator::getEstimator()
        const {
      return _estimator;
    }

    CarCalibrator::IncrementalEstimatorSP CarCalibrator::getEstimator() {
      return _estimator;
    }

    bool CarCalibrator::unprocessedMeasurements() const {
      return !_poseMeasurements.empty() || !_velocitiesMeasurements.empty() ||
        !_dmiMeasurements.empty() ||
        !_frontWheelsSpeedMeasurements.empty() ||
        !_rearWheelsSpeedMeasurements.empty() ||
        !_steeringMeasurements.empty();
    }

    const std::vector<double> CarCalibrator::getInformationGainHistory() const {
      return _infoGainHistory;
    }

    const std::vector<Eigen::VectorXd>
        CarCalibrator::getOdometryVariablesHistory() const {
      return _odometryVariablesHistory;
    }

    Eigen::VectorXd CarCalibrator::getOdometryVariablesVariance() const {
      return _estimator->getSigma2Theta().diagonal();
    }

    const CarCalibrator::TranslationSplineSP&
        CarCalibrator::getTranslationSpline() const {
      return _translationSpline;
    }

    const CarCalibrator::RotationSplineSP&
        CarCalibrator::getRotationSpline() const {
      return _rotationSpline;
    }

    const CarCalibrator::PoseMeasurements& CarCalibrator::getPoseMeasurements()
        const {
      return _poseMeasurements;
    }

    const CarCalibrator::PoseMeasurements& CarCalibrator::getPosePredictions()
        const {
      return _poseMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>& CarCalibrator::getPosePredictionErrors()
        const {
      return _poseMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getPosePredictionErrors2() const {
      return _poseMeasurementsPredErrors2;
    }

    const CarCalibrator::VelocitiesMeasurements&
        CarCalibrator::getVelocitiesMeasurements() const {
      return _velocitiesMeasurements;
    }

    const CarCalibrator::VelocitiesMeasurements&
        CarCalibrator::getVelocitiesPredictions() const {
      return _velocitiesMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        CarCalibrator::getVelocitiesPredictionErrors() const {
      return _velocitiesMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getVelocitiesPredictionErrors2()
        const {
      return _velocitiesMeasurementsPredErrors2;
    }

    const CarCalibrator::DMIMeasurements& CarCalibrator::getDMIMeasurements()
        const {
      return _dmiMeasurements;
    }

    const CarCalibrator::DMIMeasurements& CarCalibrator::getDMIPredictions()
        const {
      return _dmiMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>& CarCalibrator::getDMIPredictionErrors()
        const {
      return _dmiMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getDMIPredictionErrors2() const {
      return _dmiMeasurementsPredErrors2;
    }

    const CarCalibrator::WheelsSpeedMeasurements&
        CarCalibrator::getRearWheelsMeasurements() const {
      return _rearWheelsSpeedMeasurements;
    }

    const CarCalibrator::WheelsSpeedMeasurements&
        CarCalibrator::getRearWheelsPredictions() const {
      return _rearWheelsSpeedMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        CarCalibrator::getRearWheelsPredictionErrors() const {
      return _rearWheelsSpeedMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getRearWheelsPredictionErrors2()
        const {
      return _rearWheelsSpeedMeasurementsPredErrors2;
    }

    const CarCalibrator::WheelsSpeedMeasurements&
        CarCalibrator::getFrontWheelsMeasurements() const {
      return _frontWheelsSpeedMeasurements;
    }

    const CarCalibrator::WheelsSpeedMeasurements&
        CarCalibrator::getFrontWheelsPredictions() const {
      return _frontWheelsSpeedMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        CarCalibrator::getFrontWheelsPredictionErrors() const {
      return _frontWheelsSpeedMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getFrontWheelsPredictionErrors2()
        const {
      return _frontWheelsSpeedMeasurementsPredErrors2;
    }

    const CarCalibrator::SteeringMeasurements&
        CarCalibrator::getSteeringMeasurements() const {
      return _steeringMeasurements;
    }

    const CarCalibrator::SteeringMeasurements&
        CarCalibrator::getSteeringPredictions() const {
      return _steeringMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        CarCalibrator::getSteeringPredictionErrors() const {
      return _steeringMeasurementsPredErrors;
    }

    const std::vector<double>& CarCalibrator::getSteeringPredictionErrors2()
        const {
      return _steeringMeasurementsPredErrors2;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CarCalibrator::clearPredictions() {
      _poseMeasurementsPred.clear();
      _poseMeasurementsPredErrors.clear();
      _poseMeasurementsPredErrors2.clear();
      _velocitiesMeasurementsPred.clear();
      _velocitiesMeasurementsPredErrors.clear();
      _velocitiesMeasurementsPredErrors2.clear();
      _dmiMeasurementsPred.clear();
      _dmiMeasurementsPredErrors.clear();
      _dmiMeasurementsPredErrors2.clear();
      _frontWheelsSpeedMeasurementsPred.clear();
      _frontWheelsSpeedMeasurementsPredErrors.clear();
      _frontWheelsSpeedMeasurementsPredErrors2.clear();
      _rearWheelsSpeedMeasurementsPred.clear();
      _rearWheelsSpeedMeasurementsPredErrors.clear();
      _rearWheelsSpeedMeasurementsPredErrors2.clear();
      _steeringMeasurementsPred.clear();
      _steeringMeasurementsPredErrors.clear();
      _steeringMeasurementsPredErrors2.clear();
    }

    void CarCalibrator::predict() {
      auto batch = boost::make_shared<OptimizationProblemSpline>();
      _odometryDesignVariables->addToBatch(batch, 1);
      initSplines(_poseMeasurements);
      predictPoses(_poseMeasurements);
      predictVelocities(_velocitiesMeasurements);
      predictDMI(_dmiMeasurements);
      predictFrontWheels(_frontWheelsSpeedMeasurements);
      predictRearWheels(_rearWheelsSpeedMeasurements);
      predictSteering(_steeringMeasurements);
    }

    void CarCalibrator::addPoseMeasurement(const PoseMeasurement& pose, const
        VelocitiesMeasurement& vel, NsecTime timestamp) {
      addMeasurement(timestamp);
      _poseMeasurements.push_back(std::make_pair(timestamp, pose));
      _velocitiesMeasurements.push_back(std::make_pair(timestamp, vel));
    }

    void CarCalibrator::addDMIMeasurement(const DMIMeasurement& data, NsecTime
        timestamp) {
      addMeasurement(timestamp);
      _dmiMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addFrontWheelsMeasurement(const WheelSpeedsMeasurement&
        data, NsecTime timestamp) {
      addMeasurement(timestamp);
      _frontWheelsSpeedMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void CarCalibrator::addRearWheelsMeasurement(const WheelSpeedsMeasurement&
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
      if (_poseMeasurements.size() < 2)
        return;

      auto batch = boost::make_shared<OptimizationProblemSpline>();
      _odometryDesignVariables->addToBatch(batch, 1);
      initSplines(_poseMeasurements);
      batch->addSpline(_translationSpline, 0);
      batch->addSpline(_rotationSpline, 0);
      if (_options.useVelocities)
        addVelocitiesErrorTerms(_velocitiesMeasurements, batch);
      if (_options.usePose)
        addPoseErrorTerms(_poseMeasurements, batch);
      addDMIErrorTerms(_dmiMeasurements, batch);
      addFrontWheelsErrorTerms(_frontWheelsSpeedMeasurements, batch);
      addRearWheelsErrorTerms(_rearWheelsSpeedMeasurements, batch);
      addSteeringErrorTerms(_steeringMeasurements, batch);
      clearMeasurements();
      _currentBatchStartTimestamp = _lastTimestamp;
      batch->setGroupsOrdering({0, 1});
      if (_options.verbose) {
        std::cout << "calibration before batch: " << std::endl;
        std::cout << *_odometryDesignVariables << std::endl;
      }
      IncrementalEstimator::ReturnValue ret = _estimator->addBatch(batch);
      if (_options.verbose) {
        std::cout << "IG: " << ret.informationGain << std::endl;
        ret.batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
        std::cout << std::endl;
        std::cout << "calibration after batch: " << std::endl;
        std::cout << *_odometryDesignVariables << std::endl;
        std::cout << "singular values: " << std::endl
          << ret.singularValuesScaled << std::endl;
        std::cout << "observability: " << std::endl;
        std::cout << "e_r: " << ret.obsBasisScaled.row(0).norm() << std::endl;
        std::cout << "e_f: " << ret.obsBasisScaled.row(1).norm() << std::endl;
        std::cout << "L: " << ret.obsBasisScaled.row(2).norm() << std::endl;
        std::cout << "a0: " << ret.obsBasisScaled.row(3).norm() << std::endl;
        std::cout << "a1: " << ret.obsBasisScaled.row(4).norm() << std::endl;
        std::cout << "a2: " << ret.obsBasisScaled.row(5).norm() << std::endl;
        std::cout << "a3: " << ret.obsBasisScaled.row(6).norm() << std::endl;
        std::cout << "k_rl: " << ret.obsBasisScaled.row(7).norm() << std::endl;
        std::cout << "k_rr: " << ret.obsBasisScaled.row(8).norm() << std::endl;
        std::cout << "k_fl: " << ret.obsBasisScaled.row(9).norm() << std::endl;
        std::cout << "k_fr: " << ret.obsBasisScaled.row(10).norm() << std::endl;
        std::cout << "k_dmi: " << ret.obsBasisScaled.row(11).norm()
          << std::endl;
        std::cout << "v_r_vr_1: " << ret.obsBasisScaled.row(12).norm()
          << std::endl;
        std::cout << "v_r_vr_2: " << ret.obsBasisScaled.row(13).norm()
          << std::endl;
        std::cout << "v_r_vr_3: " << ret.obsBasisScaled.row(14).norm()
          << std::endl;
        std::cout << "v_R_r_1: " << ret.obsBasisScaled.row(15).norm()
          << std::endl;
        std::cout << "v_R_r_2: " << ret.obsBasisScaled.row(16).norm()
          << std::endl;
        std::cout << "v_R_r_3: " << ret.obsBasisScaled.row(17).norm()
          << std::endl;
      }
      _infoGainHistory.push_back(ret.informationGain);
      _odometryVariablesHistory.push_back(
        _odometryDesignVariables->getParameters());
    }

    void CarCalibrator::initSplines(const PoseMeasurements& measurements) {
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
        const Eigen::Matrix3d m_R_r =
          ypr.parametersToRotationMatrix(it->second.m_R_r);
        const Eigen::Vector4d& v_q_r =
          _odometryDesignVariables->v_R_r->getQuaternion();
        const Eigen::Matrix3d v_R_r = quat2r(v_q_r);
        const Eigen::Matrix3d m_R_v = m_R_r * v_R_r.transpose();
        Eigen::Vector4d m_q_v = r2quat(m_R_v);
        if (!rotPoses.empty()) {
          const Eigen::Vector4d lastRotPose = rotPoses.back();
          m_q_v = bestQuat(lastRotPose, m_q_v);
        }
        timestamps.push_back(timestamp);
        rotPoses.push_back(m_q_v);
        Eigen::MatrixXd v_r_vr;
        _odometryDesignVariables->v_r_vr->getParameters(v_r_vr);
        transPoses.push_back(it->second.m_r_mr - m_R_v * v_r_vr);
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
    }

    void CarCalibrator::addPoseErrorTerms(const PoseMeasurements& measurements,
        const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        ErrorTermPose::Input m_T_r;
        m_T_r.head<3>() = it->second.m_r_mr;
        m_T_r.tail<3>() = it->second.m_R_r;
        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
        Q.topLeftCorner<3, 3>() = it->second.sigma2_m_r_mr;
        Q.bottomRightCorner<3, 3>() = it->second.sigma2_m_R_r;
        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_r_vr = m_R_v * v_r_vr;
        auto m_r_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression());
        auto m_r_mr = m_r_mv + m_r_vr;
        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
        auto m_R_r = m_R_v * v_R_r;
        auto e_pose = boost::make_shared<ErrorTermPose>(
          TransformationExpression(m_R_r, m_r_mr), m_T_r, Q);
        batch->addErrorTerm(e_pose);
      }
    }

    void CarCalibrator::predictPoses(const PoseMeasurements& measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        ErrorTermPose::Input m_T_r;
        m_T_r.head<3>() = it->second.m_r_mr;
        m_T_r.tail<3>() = it->second.m_R_r;
        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
        Q.topLeftCorner<3, 3>() = it->second.sigma2_m_r_mr;
        Q.bottomRightCorner<3, 3>() = it->second.sigma2_m_R_r;
        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_r_vr = m_R_v * v_r_vr;
        auto m_r_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression());
        auto m_r_mr = m_r_mv + m_r_vr;
        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
        auto m_R_r = m_R_v * v_R_r;
        auto e_pose = boost::make_shared<ErrorTermPose>(
          TransformationExpression(m_R_r, m_r_mr), m_T_r, Q);
        auto sr = e_pose->evaluateError();
        auto error = e_pose->error();
        PoseMeasurement pose;
        pose.m_r_mr = m_r_mr.toValue();
        const EulerAnglesYawPitchRoll ypr;
        pose.m_R_r = ypr.rotationMatrixToParameters(m_R_r.toRotationMatrix());
        _poseMeasurementsPred.push_back(std::make_pair(timestamp, pose));
        _poseMeasurementsPredErrors.push_back(error);
        _poseMeasurementsPredErrors2.push_back(sr);
      }
    }

    void CarCalibrator::addVelocitiesErrorTerms(const VelocitiesMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
        auto r_v_mr = v_R_r.inverse() * (v_v_mv + v_om_mv.cross(v_r_vr));
        auto r_om_mr = v_R_r.inverse() * v_om_mv;

        auto e_vel = boost::make_shared<ErrorTermVelocities>(r_v_mr, r_om_mr,
          it->second.r_v_mr, it->second.r_om_mr, it->second.sigma2_r_v_mr,
          it->second.sigma2_r_om_mr);
        batch->addErrorTerm(e_vel);
      }
    }

    void CarCalibrator::predictVelocities(const VelocitiesMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
        auto r_v_mr = v_R_r.inverse() * (v_v_mv + v_om_mv.cross(v_r_vr));
        auto r_om_mr = v_R_r.inverse() * v_om_mv;

        auto e_vel = boost::make_shared<ErrorTermVelocities>(r_v_mr, r_om_mr,
          it->second.r_v_mr, it->second.r_om_mr, it->second.sigma2_r_v_mr,
          it->second.sigma2_r_om_mr);
        auto sr = e_vel->evaluateError();
        auto error = e_vel->error();
        VelocitiesMeasurement vel;
        vel.r_v_mr = r_v_mr.toValue();
        vel.r_om_mr = r_om_mr.toValue();
        _velocitiesMeasurementsPred.push_back(std::make_pair(timestamp, vel));
        _velocitiesMeasurementsPredErrors.push_back(error);
        _velocitiesMeasurementsPredErrors2.push_back(sr);
      }
    }

    void CarCalibrator::addDMIErrorTerms(const DMIMeasurements& measurements,
        const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw = v_v_mv + v_om_mv.cross(v_r_wl);

        auto e_dmi = boost::make_shared<ErrorTermWheel>(w_v_mw,
          ScalarExpression(_odometryDesignVariables->k_dmi),
          it->second.wheelSpeed, Eigen::Vector3d(_options.dmiVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        batch->addErrorTerm(e_dmi);
      }
    }

    void CarCalibrator::predictDMI(const DMIMeasurements& measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw = v_v_mv + v_om_mv.cross(v_r_wl);

        auto e_dmi = boost::make_shared<ErrorTermWheel>(w_v_mw,
          ScalarExpression(_odometryDesignVariables->k_dmi),
          it->second.wheelSpeed, Eigen::Vector3d(_options.dmiVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        auto sr = e_dmi->evaluateError();
        auto error = e_dmi->error();
        DMIMeasurement dmi;
        dmi.wheelSpeed = _odometryDesignVariables->k_dmi->toScalar() *
          w_v_mw.toValue()(0);
        _dmiMeasurementsPred.push_back(std::make_pair(timestamp, dmi));
        _dmiMeasurementsPredErrors.push_back(error);
        _dmiMeasurementsPredErrors2.push_back(sr);
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

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        if (v_v_mv.toValue()(0) < 0)
          continue;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_f = ScalarExpression(_odometryDesignVariables->e_f);
        auto L = ScalarExpression(_odometryDesignVariables->L);
        auto v_r_wl =
          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L +
          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
        auto v_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
        auto v_r_wr =
          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L -
          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
        auto v_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

        auto e_flw = boost::make_shared<ErrorTermWheel>(v_v_mw_l,
          ScalarExpression(_odometryDesignVariables->k_fl),
          it->second.left, Eigen::Vector3d(_options.flwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
        batch->addErrorTerm(e_flw);
          _odometryDesignVariables->k_fr->toScalar();
        auto e_frw = boost::make_shared<ErrorTermWheel>(v_v_mw_r,
          ScalarExpression(_odometryDesignVariables->k_fr),
          it->second.right, Eigen::Vector3d(_options.frwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
        batch->addErrorTerm(e_frw);
      }
    }

    void CarCalibrator::predictFrontWheels(const WheelsSpeedMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp ||
            it->second.left < _options.wheelSpeedSensorCutoff ||
            it->second.right < _options.wheelSpeedSensorCutoff)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        if (v_v_mv.toValue()(0) < 0)
          continue;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_f = ScalarExpression(_odometryDesignVariables->e_f);
        auto L = ScalarExpression(_odometryDesignVariables->L);
        auto v_r_wl =
          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L +
          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
        auto v_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
        auto v_r_wr =
          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L -
          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
        auto v_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

        auto e_flw = boost::make_shared<ErrorTermWheel>(v_v_mw_l,
          ScalarExpression(_odometryDesignVariables->k_fl),
          it->second.left, Eigen::Vector3d(_options.flwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
          _odometryDesignVariables->k_fr->toScalar();
        auto e_frw = boost::make_shared<ErrorTermWheel>(v_v_mw_r,
          ScalarExpression(_odometryDesignVariables->k_fr),
          it->second.right, Eigen::Vector3d(_options.frwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
        auto sr_l = e_flw->evaluateError();
        auto error_l = e_flw->error();
        auto sr_r = e_frw->evaluateError();
        auto error_r = e_frw->error();
        double v0 = v_v_mw_l.toValue()(0);
        double v1 = v_v_mw_l.toValue()(1);
        double k = _odometryDesignVariables->k_fl->toScalar();
        double temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        WheelSpeedsMeasurement data;
        data.left = k * (v0 / temp + v1 * v1 / (v0 * temp));
        v0 = v_v_mw_r.toValue()(0);
        v1 = v_v_mw_r.toValue()(1);
        k = _odometryDesignVariables->k_fr->toScalar();
        temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        data.right = k * (v0 / temp + v1 * v1 / (v0 * temp));
        _frontWheelsSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
          data));
        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = error_l;
        error.tail<3>() = error_r;
        _frontWheelsSpeedMeasurementsPredErrors.push_back(error);
        _frontWheelsSpeedMeasurementsPredErrors2.push_back(sr_l + sr_r);
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

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        if (v_v_mv.toValue()(0) < 0)
          continue;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
        auto v_r_wr =
          -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

        auto e_rlw = boost::make_shared<ErrorTermWheel>(w_v_mw_l,
          ScalarExpression(_odometryDesignVariables->k_rl),
          it->second.left, Eigen::Vector3d(_options.flwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        batch->addErrorTerm(e_rlw);
        auto e_rrw = boost::make_shared<ErrorTermWheel>(w_v_mw_r,
          ScalarExpression(_odometryDesignVariables->k_rr),
          it->second.right, Eigen::Vector3d(_options.frwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        batch->addErrorTerm(e_rrw);
      }
    }

    void CarCalibrator::predictRearWheels(const WheelsSpeedMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp ||
            it->second.left < _options.wheelSpeedSensorCutoff ||
            it->second.right < _options.wheelSpeedSensorCutoff)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        if (v_v_mv.toValue()(0) < 0)
          continue;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
        auto v_r_wr =
          -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
        auto w_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

        auto e_rlw = boost::make_shared<ErrorTermWheel>(w_v_mw_l,
          ScalarExpression(_odometryDesignVariables->k_rl),
          it->second.left, Eigen::Vector3d(_options.flwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        auto e_rrw = boost::make_shared<ErrorTermWheel>(w_v_mw_r,
          ScalarExpression(_odometryDesignVariables->k_rr),
          it->second.right, Eigen::Vector3d(_options.frwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal());
        auto sr_l = e_rlw->evaluateError();
        auto error_l = e_rlw->error();
        auto sr_r = e_rrw->evaluateError();
        auto error_r = e_rrw->error();
        WheelSpeedsMeasurement data;
        data.left = _odometryDesignVariables->k_rl->toScalar() *
          w_v_mw_l.toValue()(0);
        data.right = _odometryDesignVariables->k_rr->toScalar() *
          w_v_mw_r.toValue()(0);
        _rearWheelsSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
          data));
        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = error_l;
        error.tail<3>() = error_r;
        _rearWheelsSpeedMeasurementsPredErrors.push_back(error);
        _rearWheelsSpeedMeasurementsPredErrors2.push_back(sr_l + sr_r);
      }
    }

    void CarCalibrator::addSteeringErrorTerms(const SteeringMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto L = ScalarExpression(_odometryDesignVariables->L);
        auto v_r_w = EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L;
        auto v_v_mw = v_v_mv + v_om_mv.cross(v_r_w);

        if (std::fabs(v_v_mw.toValue()(0)) < _options.linearVelocityTolerance)
          continue;

        auto e_st = boost::make_shared<ErrorTermSteering>(v_v_mw,
          it->second.value, _options.steeringVariance,
          _odometryDesignVariables->a.get());
        batch->addErrorTerm(e_st);
      }
    }

    void CarCalibrator::predictSteering(const SteeringMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        if (_translationSpline->getMinTime() > timestamp ||
            _translationSpline->getMaxTime() < timestamp)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto m_v_mv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_mv = m_R_v.inverse() * m_v_mv;
        auto m_om_mv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_mv = m_R_v.inverse() * m_om_mv;
        auto L = ScalarExpression(_odometryDesignVariables->L);
        auto v_r_w = EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L;
        auto v_v_mw = v_v_mv + v_om_mv.cross(v_r_w);

        if (std::fabs(v_v_mw.toValue()(0)) < _options.linearVelocityTolerance)
          continue;

        auto e_st = boost::make_shared<ErrorTermSteering>(v_v_mw,
          it->second.value, _options.steeringVariance,
          _odometryDesignVariables->a.get());
        auto sr = e_st->evaluateError();
        auto error = e_st->error();
        SteeringMeasurement data;
        const double phi = atan2(v_v_mw.toValue()(1), v_v_mw.toValue()(0));
        data.value = phi;
        _steeringMeasurementsPred.push_back(std::make_pair(timestamp, data));
        _steeringMeasurementsPredErrors.push_back(error);
        _steeringMeasurementsPredErrors2.push_back(sr);
      }
    }

    void CarCalibrator::clearMeasurements() {
      _poseMeasurements.clear();
      _velocitiesMeasurements.clear();
      _dmiMeasurements.clear();
      _frontWheelsSpeedMeasurements.clear();
      _rearWheelsSpeedMeasurements.clear();
      _steeringMeasurements.clear();
    }

  }
}
