/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
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
#include <aslam/backend/GenericScalar.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>

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
        std::cout << "t_l: " << ret.obsBasisScaled.row(2).norm() << std::endl;
        std::cout << "k_r: " << ret.obsBasisScaled.row(3).norm() << std::endl;
        std::cout << "t_r: " << ret.obsBasisScaled.row(4).norm() << std::endl;
        std::cout << "v_r_vp_1: " << ret.obsBasisScaled.row(5).norm()
          << std::endl;
        std::cout << "v_r_vp_2: " << ret.obsBasisScaled.row(6).norm()
          << std::endl;
        std::cout << "v_r_vp_3: " << ret.obsBasisScaled.row(7).norm()
          << std::endl;
        std::cout << "v_R_p_1: " << ret.obsBasisScaled.row(8).norm()
          << std::endl;
        std::cout << "v_R_p_2: " << ret.obsBasisScaled.row(9).norm()
          << std::endl;
        std::cout << "v_R_p_3: " << ret.obsBasisScaled.row(10).norm()
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
        const Eigen::Matrix3d w_R_p =
          ypr.parametersToRotationMatrix(it->second.w_R_p);
        const Eigen::Vector4d& v_q_p =
          _odometryDesignVariables->v_R_p->getQuaternion();
        const Eigen::Matrix3d v_R_p = quat2r(v_q_p);
        const Eigen::Matrix3d w_R_v = w_R_p * v_R_p.transpose();
        Eigen::Vector4d w_q_v = r2quat(w_R_v);
        if (!rotPoses.empty()) {
          const Eigen::Vector4d lastRotPose = rotPoses.back();
          w_q_v = bestQuat(lastRotPose, w_q_v);
        }
        timestamps.push_back(timestamp);
        rotPoses.push_back(w_q_v);
        Eigen::MatrixXd v_r_vp;
        _odometryDesignVariables->v_r_vp->getParameters(v_r_vp);
        transPoses.push_back(it->second.w_r_wp - w_R_v * v_r_vp);
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
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        ErrorTermPose::Input w_T_p;
        w_T_p.head<3>() = it->second.w_r_wp;
        w_T_p.tail<3>() = it->second.w_R_p;
        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
        Q.topLeftCorner<3, 3>() = it->second.sigma2_w_r_wp;
        Q.bottomRightCorner<3, 3>() = it->second.sigma2_w_R_p;
        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

        auto v_r_vp = EuclideanExpression(_odometryDesignVariables->v_r_vp);
        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_r_vp = w_R_v * v_r_vp;
        auto w_r_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression());
        auto w_r_wp = w_r_wv + w_r_vp;
        auto v_R_p = RotationExpression(_odometryDesignVariables->v_R_p);
        auto w_R_p = w_R_v * v_R_p;
        auto e_pose = boost::make_shared<ErrorTermPose>(
          TransformationExpression(w_R_p, w_r_wp), w_T_p, Q);
        batch->addErrorTerm(e_pose);
      }
    }

    void Calibrator::predictPoses(const PoseMeasurements& measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        ErrorTermPose::Input w_T_p;
        w_T_p.head<3>() = it->second.w_r_wp;
        w_T_p.tail<3>() = it->second.w_R_p;
        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
        Q.topLeftCorner<3, 3>() = it->second.sigma2_w_r_wp;
        Q.bottomRightCorner<3, 3>() = it->second.sigma2_w_R_p;
        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<0>(timestamp);

        auto v_r_vp = EuclideanExpression(_odometryDesignVariables->v_r_vp);
        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_r_vp = w_R_v * v_r_vp;
        auto w_r_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression());
        auto w_r_wp = w_r_wv + w_r_vp;
        auto v_R_p = RotationExpression(_odometryDesignVariables->v_R_p);
        auto w_R_p = w_R_v * v_R_p;
        auto e_pose = boost::make_shared<ErrorTermPose>(
          TransformationExpression(w_R_p, w_r_wp), w_T_p, Q);
        auto sr = e_pose->evaluateError();
        auto error = e_pose->error();
        PoseMeasurement pose;
        pose.w_r_wp = w_r_wp.toValue();
        const EulerAnglesYawPitchRoll ypr;
        pose.w_R_p = ypr.rotationMatrixToParameters(w_R_p.toRotationMatrix());
        _poseMeasurementsPred.push_back(std::make_pair(timestamp, pose));
        _poseMeasurementsPredErrors.push_back(error);
        _poseMeasurementsPredErrors2.push_back(sr);
      }
    }

    void Calibrator::addLeftWheelErrorTerms(const WheelSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        auto timeDelay = _odometryDesignVariables->t_l->toExpression();
        auto timestampDelay = timeDelay +
          GenericScalarExpression<OdometryDesignVariables::Time>(timestamp);
        auto Tmax = _translationSpline->getMaxTime();
        auto Tmin = _translationSpline->getMinTime();
        auto lBound = -_options.delayBound +
          timestampDelay.toScalar().getNumerator();
        auto uBound = _options.delayBound +
          timestampDelay.toScalar().getNumerator();

        if(uBound > Tmax || lBound < Tmin)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);

        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_v_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_wv = w_R_v.inverse() * w_v_wv;
        auto w_om_wv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_wv = w_R_v.inverse() * w_om_wv;
        auto b = ScalarExpression(_odometryDesignVariables->b);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * b;
        auto v_v_wl = v_v_wv + v_om_wv.cross(v_r_wl);

        auto e_lw = boost::make_shared<ErrorTermWheel>(v_v_wl,
          ScalarExpression(_odometryDesignVariables->k_l),
          it->second.value, Eigen::Vector3d(_options.lwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), false);
        batch->addErrorTerm(e_lw);
      }
    }

    void Calibrator::predictLeftWheel(const WheelSpeedMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        auto timeDelay = _odometryDesignVariables->t_l->toExpression();
        auto timestampDelay = timeDelay +
          GenericScalarExpression<OdometryDesignVariables::Time>(timestamp);
        auto Tmax = _translationSpline->getMaxTime();
        auto Tmin = _translationSpline->getMinTime();
        auto lBound = -_options.delayBound +
          timestampDelay.toScalar().getNumerator();
        auto uBound = _options.delayBound +
          timestampDelay.toScalar().getNumerator();

        if(uBound > Tmax || lBound < Tmin)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);

        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_v_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_wv = w_R_v.inverse() * w_v_wv;
        auto w_om_wv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_wv = w_R_v.inverse() * w_om_wv;
        auto b = ScalarExpression(_odometryDesignVariables->b);
        auto v_r_wl = EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * b;
        auto v_v_wl = v_v_wv + v_om_wv.cross(v_r_wl);

        auto e_lw = boost::make_shared<ErrorTermWheel>(v_v_wl,
          ScalarExpression(_odometryDesignVariables->k_l),
          it->second.value, Eigen::Vector3d(_options.lwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), false);
        auto sr_l = e_lw->evaluateError();
        auto error_l = e_lw->error();
        double v0 = v_v_wl.toValue()(0);
        double v1 = v_v_wl.toValue()(1);
        double k = _odometryDesignVariables->k_l->toScalar();
        double temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        WheelSpeedMeasurement data;
        data.value = k * (v0 / temp + v1 * v1 / (v0 * temp));
        _leftWheelSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
          data));
        _leftWheelSpeedMeasurementsPredErrors.push_back(error_l);
        _leftWheelSpeedMeasurementsPredErrors2.push_back(sr_l);
      }
    }

    void Calibrator::addRightWheelErrorTerms(const WheelSpeedMeasurements&
        measurements, const OptimizationProblemSplineSP& batch) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        auto timeDelay = _odometryDesignVariables->t_r->toExpression();
        auto timestampDelay = timeDelay +
          GenericScalarExpression<OdometryDesignVariables::Time>(timestamp);
        auto Tmax = _translationSpline->getMaxTime();
        auto Tmin = _translationSpline->getMinTime();
        auto lBound = -_options.delayBound +
          timestampDelay.toScalar().getNumerator();
        auto uBound = _options.delayBound +
          timestampDelay.toScalar().getNumerator();

        if(uBound > Tmax || lBound < Tmin)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);

        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_v_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_wv = w_R_v.inverse() * w_v_wv;
        auto w_om_wv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_wv = w_R_v.inverse() * w_om_wv;
        auto b = ScalarExpression(_odometryDesignVariables->b);
        auto v_r_wr =
          -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * b;
        auto v_v_wr = v_v_wv + v_om_wv.cross(v_r_wr);

        auto e_rw = boost::make_shared<ErrorTermWheel>(v_v_wr,
          ScalarExpression(_odometryDesignVariables->k_r),
          it->second.value, Eigen::Vector3d(_options.rwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), false);
        batch->addErrorTerm(e_rw);
      }
    }

    void Calibrator::predictRightWheel(const WheelSpeedMeasurements&
        measurements) {
      for (auto it = measurements.cbegin(); it != measurements.cend(); ++it) {
        auto timestamp = it->first;
        auto timeDelay = _odometryDesignVariables->t_r->toExpression();
        auto timestampDelay = timeDelay +
          GenericScalarExpression<OdometryDesignVariables::Time>(timestamp);
        auto Tmax = _translationSpline->getMaxTime();
        auto Tmin = _translationSpline->getMinTime();
        auto lBound = -_options.delayBound +
          timestampDelay.toScalar().getNumerator();
        auto uBound = _options.delayBound +
          timestampDelay.toScalar().getNumerator();

        if(uBound > Tmax || lBound < Tmin)
          continue;

        auto translationExpressionFactory =
          _translationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);
        auto rotationExpressionFactory =
          _rotationSpline->getExpressionFactoryAt<1>(timestampDelay,
          lBound, uBound);

        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
          rotationExpressionFactory.getValueExpression());
        auto w_v_wv = EuclideanExpression(
          translationExpressionFactory.getValueExpression(1));
        auto v_v_wv = w_R_v.inverse() * w_v_wv;
        auto w_om_wv = -EuclideanExpression(
          rotationExpressionFactory.getAngularVelocityExpression());
        auto v_om_wv = w_R_v.inverse() * w_om_wv;
        auto b = ScalarExpression(_odometryDesignVariables->b);
        auto v_r_wr = -EuclideanExpression(Eigen::Vector3d(0.0, 1.0, 0.0)) * b;
        auto v_v_wr = v_v_wv + v_om_wv.cross(v_r_wr);

        auto e_rw = boost::make_shared<ErrorTermWheel>(v_v_wr,
          ScalarExpression(_odometryDesignVariables->k_r),
          it->second.value, Eigen::Vector3d(_options.rwVariance,
          _options.vyVariance, _options.vzVariance).asDiagonal(), false);
        auto sr_r = e_rw->evaluateError();
        auto error_r = e_rw->error();
        double v0 = v_v_wr.toValue()(0);
        double v1 = v_v_wr.toValue()(1);
        double k = _odometryDesignVariables->k_r->toScalar();
        double temp = std::sqrt(v1 * v1 / (v0 * v0) + 1);
        WheelSpeedMeasurement data;
        data.value = k * (v0 / temp + v1 * v1 / (v0 * temp));
        _rightWheelSpeedMeasurementsPred.push_back(std::make_pair(timestamp,
          data));
        _rightWheelSpeedMeasurementsPredErrors.push_back(error_r);
        _rightWheelSpeedMeasurementsPredErrors2.push_back(sr_r);
      }
    }

    void Calibrator::clearMeasurements() {
      _poseMeasurements.clear();
      _leftWheelSpeedMeasurements.clear();
      _rightWheelSpeedMeasurements.clear();
    }

  }
}
