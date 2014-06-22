/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

#include "aslam/calibration/time-delay/algo/Calibrator.h"

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

#include "aslam/calibration/time-delay/error-terms/ErrorTermPose.h"
#include "aslam/calibration/time-delay/error-terms/ErrorTermWheel.h"
#include "aslam/calibration/time-delay/design-variables/OdometryDesignVariables.h"
#include "aslam/calibration/time-delay/algo/OptimizationProblemSpline.h"
#include "aslam/calibration/time-delay/algo/bestQuat.h"
#include "aslam/calibration/time-delay/data/PoseMeasurement.h"
#include "aslam/calibration/time-delay/data/WheelSpeedMeasurement.h"

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

    Calibrator::Calibrator(const PropertyTree& config) :
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

    Calibrator::~Calibrator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const Calibrator::Options& Calibrator::getOptions() const {
      return _options;
    }

    Calibrator::Options& Calibrator::getOptions() {
      return _options;
    }

    const Calibrator::OdometryDesignVariablesSP&
        Calibrator::getOdometryDesignVariables() const {
      return _odometryDesignVariables;
    }

    Calibrator::OdometryDesignVariablesSP&
        Calibrator::getOdometryDesignVariables() {
      return _odometryDesignVariables;
    }

    const Calibrator::IncrementalEstimatorSP Calibrator::getEstimator() const {
      return _estimator;
    }

    Calibrator::IncrementalEstimatorSP Calibrator::getEstimator() {
      return _estimator;
    }

    bool Calibrator::unprocessedMeasurements() const {
      return !_poseMeasurements.empty() ||
        !_leftWheelSpeedMeasurements.empty() ||
        !_rightWheelSpeedMeasurements.empty();
    }

    const std::vector<double> Calibrator::getInformationGainHistory() const {
      return _infoGainHistory;
    }

    const std::vector<Eigen::VectorXd>
        Calibrator::getOdometryVariablesHistory() const {
      return _odometryVariablesHistory;
    }

    Eigen::VectorXd Calibrator::getOdometryVariablesVariance() const {
      return _estimator->getSigma2Theta().diagonal();
    }

    const Calibrator::TranslationSplineSP&
        Calibrator::getTranslationSpline() const {
      return _translationSpline;
    }

    const Calibrator::RotationSplineSP&
        Calibrator::getRotationSpline() const {
      return _rotationSpline;
    }

    const Calibrator::PoseMeasurements& Calibrator::getPoseMeasurements()
        const {
      return _poseMeasurements;
    }

    const Calibrator::PoseMeasurements& Calibrator::getPosePredictions() const {
      return _poseMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>& Calibrator::getPosePredictionErrors()
        const {
      return _poseMeasurementsPredErrors;
    }

    const std::vector<double>& Calibrator::getPosePredictionErrors2() const {
      return _poseMeasurementsPredErrors2;
    }

    const Calibrator::WheelSpeedMeasurements&
        Calibrator::getLeftWheelMeasurements() const {
      return _leftWheelSpeedMeasurements;
    }

    const Calibrator::WheelSpeedMeasurements&
        Calibrator::getLeftWheelPredictions() const {
      return _leftWheelSpeedMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        Calibrator::getLeftWheelPredictionErrors() const {
      return _leftWheelSpeedMeasurementsPredErrors;
    }

    const std::vector<double>& Calibrator::getLeftWheelPredictionErrors2()
        const {
      return _leftWheelSpeedMeasurementsPredErrors2;
    }

    const Calibrator::WheelSpeedMeasurements&
        Calibrator::getRightWheelMeasurements() const {
      return _rightWheelSpeedMeasurements;
    }

    const Calibrator::WheelSpeedMeasurements&
        Calibrator::getRightWheelPredictions() const {
      return _rightWheelSpeedMeasurementsPred;
    }

    const std::vector<Eigen::VectorXd>&
        Calibrator::getRightWheelPredictionErrors() const {
      return _rightWheelSpeedMeasurementsPredErrors;
    }

    const std::vector<double>& Calibrator::getRightWheelPredictionErrors2()
        const {
      return _rightWheelSpeedMeasurementsPredErrors2;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Calibrator::clearPredictions() {
      _poseMeasurementsPred.clear();
      _poseMeasurementsPredErrors.clear();
      _poseMeasurementsPredErrors2.clear();
      _leftWheelSpeedMeasurementsPred.clear();
      _leftWheelSpeedMeasurementsPredErrors.clear();
      _leftWheelSpeedMeasurementsPredErrors2.clear();
      _rightWheelSpeedMeasurementsPred.clear();
      _rightWheelSpeedMeasurementsPredErrors.clear();
      _rightWheelSpeedMeasurementsPredErrors2.clear();
    }

    void Calibrator::predict() {
      auto batch = boost::make_shared<OptimizationProblemSpline>();
      _odometryDesignVariables->addToBatch(batch, 1);
      initSplines(_poseMeasurements);
      predictPoses(_poseMeasurements);
      predictLeftWheel(_leftWheelSpeedMeasurements);
      predictRightWheel(_rightWheelSpeedMeasurements);
    }

    void Calibrator::addPoseMeasurement(const PoseMeasurement& pose, NsecTime
        timestamp) {
      addMeasurement(timestamp);
      _poseMeasurements.push_back(std::make_pair(timestamp, pose));
    }

    void Calibrator::addLeftWheelMeasurement(const WheelSpeedMeasurement& data,
        NsecTime timestamp) {
      addMeasurement(timestamp);
      _leftWheelSpeedMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void Calibrator::addRightWheelMeasurement(const WheelSpeedMeasurement& data,
        NsecTime timestamp) {
      addMeasurement(timestamp);
      _rightWheelSpeedMeasurements.push_back(std::make_pair(timestamp, data));
    }

    void Calibrator::addMeasurement(NsecTime timestamp) {
      _lastTimestamp = timestamp;
      if (_currentBatchStartTimestamp == -1)
        _currentBatchStartTimestamp = timestamp;
      if (nsecToSec(timestamp - _currentBatchStartTimestamp) >=
          _options.windowDuration)
        addMeasurements();
    }

    void Calibrator::addMeasurements() {
      if (_poseMeasurements.size() < 2)
        return;

      auto batch = boost::make_shared<OptimizationProblemSpline>();
      _odometryDesignVariables->addToBatch(batch, 1);
      initSplines(_poseMeasurements);
      batch->addSpline(_translationSpline, 0);
      batch->addSpline(_rotationSpline, 0);
      addPoseErrorTerms(_poseMeasurements, batch);
      addLeftWheelErrorTerms(_leftWheelSpeedMeasurements, batch);
      addRightWheelErrorTerms(_rightWheelSpeedMeasurements, batch);
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
        std::cout << "b: " << ret.obsBasisScaled.row(0).norm() << std::endl;
        std::cout << "k_l: " << ret.obsBasisScaled.row(1).norm() << std::endl;
        std::cout << "k_r: " << ret.obsBasisScaled.row(2).norm() << std::endl;
        std::cout << "v_r_vp_1: " << ret.obsBasisScaled.row(3).norm()
          << std::endl;
        std::cout << "v_r_vp_2: " << ret.obsBasisScaled.row(4).norm()
          << std::endl;
        std::cout << "v_r_vp_3: " << ret.obsBasisScaled.row(5).norm()
          << std::endl;
        std::cout << "v_R_p_1: " << ret.obsBasisScaled.row(6).norm()
          << std::endl;
        std::cout << "v_R_p_2: " << ret.obsBasisScaled.row(7).norm()
          << std::endl;
        std::cout << "v_R_p_3: " << ret.obsBasisScaled.row(8).norm()
          << std::endl;
      }
      _infoGainHistory.push_back(ret.informationGain);
      _odometryVariablesHistory.push_back(
        _odometryDesignVariables->getParameters());
    }

    void Calibrator::initSplines(const PoseMeasurements& measurements) {
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

    void Calibrator::addPoseErrorTerms(const PoseMeasurements& measurements,
        const OptimizationProblemSplineSP& batch) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        ErrorTermPose::Input m_T_r;
//        m_T_r.head<3>() = it->second.m_r_mr;
//        m_T_r.tail<3>() = it->second.m_R_r;
//        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
//        Q.topLeftCorner<3, 3>() = it->second.sigma2_m_r_mr;
//        Q.bottomRightCorner<3, 3>() = it->second.sigma2_m_R_r;
//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<0>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

//        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_r_vr = m_R_v * v_r_vr;
//        auto m_r_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression());
//        auto m_r_mr = m_r_mv + m_r_vr;
//        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
//        auto m_R_r = m_R_v * v_R_r;
//        auto e_pose = boost::make_shared<ErrorTermPose>(
//          TransformationExpression(m_R_r, m_r_mr), m_T_r, Q);
//        batch->addErrorTerm(e_pose);
//      }
    }

    void Calibrator::predictPoses(const PoseMeasurements& measurements) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        ErrorTermPose::Input m_T_r;
//        m_T_r.head<3>() = it->second.m_r_mr;
//        m_T_r.tail<3>() = it->second.m_R_r;
//        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
//        Q.topLeftCorner<3, 3>() = it->second.sigma2_m_r_mr;
//        Q.bottomRightCorner<3, 3>() = it->second.sigma2_m_R_r;
//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<0>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

//        auto v_r_vr = EuclideanExpression(_odometryDesignVariables->v_r_vr);
//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_r_vr = m_R_v * v_r_vr;
//        auto m_r_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression());
//        auto m_r_mr = m_r_mv + m_r_vr;
//        auto v_R_r = RotationExpression(_odometryDesignVariables->v_R_r);
//        auto m_R_r = m_R_v * v_R_r;
//        auto e_pose = boost::make_shared<ErrorTermPose>(
//          TransformationExpression(m_R_r, m_r_mr), m_T_r, Q);
//        auto sr = e_pose->evaluateError();
//        auto error = e_pose->error();
//        PoseMeasurement pose;
//        pose.m_r_mr = m_r_mr.toValue();
//        const EulerAnglesYawPitchRoll ypr;
//        pose.m_R_r = ypr.rotationMatrixToParameters(m_R_r.toRotationMatrix());
//        _poseMeasurementsPred.push_back(std::make_pair(timestamp, pose));
//        _poseMeasurementsPredErrors.push_back(error);
//        _poseMeasurementsPredErrors2.push_back(sr);
//      }
    }

    void Calibrator::addLeftWheelErrorTerms(const WheelSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        if (_translationSpline->getMinTime() > timestamp ||
//            _translationSpline->getMaxTime() < timestamp ||
//            it->second.left < _options.wheelSpeedSensorCutoff ||
//            it->second.right < _options.wheelSpeedSensorCutoff)
//          continue;

//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<1>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_v_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression(1));
//        auto v_v_mv = m_R_v.inverse() * m_v_mv;
//        if (v_v_mv.toValue()(0) < 0)
//          continue;
//        auto m_om_mv = -EuclideanExpression(
//          rotationExpressionFactory.getAngularVelocityExpression());
//        auto v_om_mv = m_R_v.inverse() * m_om_mv;
//        auto e_f = ScalarExpression(_odometryDesignVariables->e_f);
//        auto L = ScalarExpression(_odometryDesignVariables->L);
//        auto v_r_wl =
//          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L +
//          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
//        auto v_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
//        auto v_r_wr =
//          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L -
//          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
//        auto v_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

//        auto e_flw = boost::make_shared<ErrorTermWheel>(v_v_mw_l,
//          ScalarExpression(_odometryDesignVariables->k_fl),
//          it->second.left, Eigen::Vector3d(_options.flwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
//        batch->addErrorTerm(e_flw);
//          _odometryDesignVariables->k_fr->toScalar();
//        auto e_frw = boost::make_shared<ErrorTermWheel>(v_v_mw_r,
//          ScalarExpression(_odometryDesignVariables->k_fr),
//          it->second.right, Eigen::Vector3d(_options.frwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
//        batch->addErrorTerm(e_frw);
//      }
    }

    void Calibrator::predictLeftWheels(const WheelSpeedMeasurements&
        measurements) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        if (_translationSpline->getMinTime() > timestamp ||
//            _translationSpline->getMaxTime() < timestamp ||
//            it->second.left < _options.wheelSpeedSensorCutoff ||
//            it->second.right < _options.wheelSpeedSensorCutoff)
//          continue;

//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<1>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_v_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression(1));
//        auto v_v_mv = m_R_v.inverse() * m_v_mv;
//        if (v_v_mv.toValue()(0) < 0)
//          continue;
//        auto m_om_mv = -EuclideanExpression(
//          rotationExpressionFactory.getAngularVelocityExpression());
//        auto v_om_mv = m_R_v.inverse() * m_om_mv;
//        auto e_f = ScalarExpression(_odometryDesignVariables->e_f);
//        auto L = ScalarExpression(_odometryDesignVariables->L);
//        auto v_r_wl =
//          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L +
//          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
//        auto v_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
//        auto v_r_wr =
//          EuclideanExpression(Eigen::Vector3d(1.0, 0.0, 0.0)) * L -
//          EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_f;
//        auto v_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

//        auto e_flw = boost::make_shared<ErrorTermWheel>(v_v_mw_l,
//          ScalarExpression(_odometryDesignVariables->k_fl),
//          it->second.left, Eigen::Vector3d(_options.flwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
//          _odometryDesignVariables->k_fr->toScalar();
//        auto e_frw = boost::make_shared<ErrorTermWheel>(v_v_mw_r,
//          ScalarExpression(_odometryDesignVariables->k_fr),
//          it->second.right, Eigen::Vector3d(_options.frwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal(), true);
//        auto sr_l = e_flw->evaluateError();
//        auto error_l = e_flw->error();
//        auto sr_r = e_frw->evaluateError();
//        auto error_r = e_frw->error();
//        double v0 = v_v_mw_l.toValue()(0);
//        double v1 = v_v_mw_l.toValue()(1);
//        double k = _odometryDesignVariables->k_fl->toScalar();
//        double temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
//        WheelSpeedsMeasurement data;
//        data.left = k * (v0 / temp + v1 * v1 / (v0 * temp));
//        v0 = v_v_mw_r.toValue()(0);
//        v1 = v_v_mw_r.toValue()(1);
//        k = _odometryDesignVariables->k_fr->toScalar();
//        temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
//        data.right = k * (v0 / temp + v1 * v1 / (v0 * temp));
//        _frontWheelsSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
//          data));
//        Eigen::Matrix<double, 6, 1> error;
//        error.head<3>() = error_l;
//        error.tail<3>() = error_r;
//        _frontWheelsSpeedMeasurementsPredErrors.push_back(error);
//        _frontWheelsSpeedMeasurementsPredErrors2.push_back(sr_l + sr_r);
//      }
    }

    void Calibrator::addRightWheelErrorTerms(const WheelSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        if (_translationSpline->getMinTime() > timestamp ||
//            _translationSpline->getMaxTime() < timestamp ||
//            it->second.left < _options.wheelSpeedSensorCutoff ||
//            it->second.right < _options.wheelSpeedSensorCutoff)
//          continue;

//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<1>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_v_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression(1));
//        auto v_v_mv = m_R_v.inverse() * m_v_mv;
//        if (v_v_mv.toValue()(0) < 0)
//          continue;
//        auto m_om_mv = -EuclideanExpression(
//          rotationExpressionFactory.getAngularVelocityExpression());
//        auto v_om_mv = m_R_v.inverse() * m_om_mv;
//        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
//        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
//        auto w_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
//        auto v_r_wr =
//          -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
//        auto w_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

//        auto e_rlw = boost::make_shared<ErrorTermWheel>(w_v_mw_l,
//          ScalarExpression(_odometryDesignVariables->k_rl),
//          it->second.left, Eigen::Vector3d(_options.flwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal());
//        batch->addErrorTerm(e_rlw);
//        auto e_rrw = boost::make_shared<ErrorTermWheel>(w_v_mw_r,
//          ScalarExpression(_odometryDesignVariables->k_rr),
//          it->second.right, Eigen::Vector3d(_options.frwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal());
//        batch->addErrorTerm(e_rrw);
//      }
    }

    void Calibrator::predictRightWheel(const WheelSpeedMeasurements&
        measurements) {
//      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
//        auto timestamp = it->first;
//        if (_translationSpline->getMinTime() > timestamp ||
//            _translationSpline->getMaxTime() < timestamp ||
//            it->second.left < _options.wheelSpeedSensorCutoff ||
//            it->second.right < _options.wheelSpeedSensorCutoff)
//          continue;

//        auto translationExpressionFactory =
//          _translationSpline->getExpressionFactoryAt<1>(timestamp);
//        auto rotationExpressionFactory =
//          _rotationSpline->getExpressionFactoryAt<1>(timestamp);

//        auto m_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto m_v_mv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression(1));
//        auto v_v_mv = m_R_v.inverse() * m_v_mv;
//        if (v_v_mv.toValue()(0) < 0)
//          continue;
//        auto m_om_mv = -EuclideanExpression(
//          rotationExpressionFactory.getAngularVelocityExpression());
//        auto v_om_mv = m_R_v.inverse() * m_om_mv;
//        auto e_r = ScalarExpression(_odometryDesignVariables->e_r);
//        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
//        auto w_v_mw_l = v_v_mv + v_om_mv.cross(v_r_wl);
//        auto v_r_wr =
//          -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * e_r;
//        auto w_v_mw_r = v_v_mv + v_om_mv.cross(v_r_wr);

//        auto e_rlw = boost::make_shared<ErrorTermWheel>(w_v_mw_l,
//          ScalarExpression(_odometryDesignVariables->k_rl),
//          it->second.left, Eigen::Vector3d(_options.flwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal());
//        auto e_rrw = boost::make_shared<ErrorTermWheel>(w_v_mw_r,
//          ScalarExpression(_odometryDesignVariables->k_rr),
//          it->second.right, Eigen::Vector3d(_options.frwVariance,
//          _options.vyVariance, _options.vzVariance).asDiagonal());
//        auto sr_l = e_rlw->evaluateError();
//        auto error_l = e_rlw->error();
//        auto sr_r = e_rrw->evaluateError();
//        auto error_r = e_rrw->error();
//        WheelSpeedsMeasurement data;
//        data.left = _odometryDesignVariables->k_rl->toScalar() *
//          w_v_mw_l.toValue()(0);
//        data.right = _odometryDesignVariables->k_rr->toScalar() *
//          w_v_mw_r.toValue()(0);
//        _rearWheelsSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
//          data));
//        Eigen::Matrix<double, 6, 1> error;
//        error.head<3>() = error_l;
//        error.tail<3>() = error_r;
//        _rearWheelsSpeedMeasurementsPredErrors.push_back(error);
//        _rearWheelsSpeedMeasurementsPredErrors2.push_back(sr_l + sr_r);
//      }
    }

    void Calibrator::clearMeasurements() {
      _poseMeasurements.clear();
      _leftWheelSpeedMeasurements.clear();
      _rightWheelSpeedMeasurements.clear();
    }

  }
}
