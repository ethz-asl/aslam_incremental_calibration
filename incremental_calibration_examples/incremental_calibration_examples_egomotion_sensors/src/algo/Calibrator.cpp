/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
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

#include "aslam/calibration/egomotion/algo/Calibrator.h"

#include <cmath>

#include <boost/make_shared.hpp>

#include <sm/kinematics/Transformation.hpp>

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
#include <aslam/backend/ErrorTermTransformation.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>

#include "aslam/calibration/egomotion/algo/OptimizationProblemSpline.h"
#include "aslam/calibration/egomotion/algo/bestQuat.h"

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
        currentBatchStartTimestamp_(-1),
        lastTimestamp_(-1) {
      // create the underlying estimator
      estimator_ = boost::make_shared<IncrementalEstimator>(
        sm::PropertyTree(config, "estimator"));

      // sets the options for the calibrator
      options_ = Options(config);
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Calibrator::clearPredictions() {
      motionMeasurementsPred_.clear();
      motionMeasurementsPredErrors_.clear();
      motionMeasurementsPredErrors2_.clear();
    }

    void Calibrator::predict() {
      initSplines(options_.referenceSensor);
      for (const auto& measurements : motionMeasurements_)
        predictMotion(measurements.first);
    }

    void Calibrator::addMotionMeasurement(const MotionMeasurement& motion,
        sm::timing::NsecTime timestamp, size_t idx) {
      addMeasurement(timestamp);
      motionMeasurements_[idx].push_back(std::make_pair(timestamp,
          motion));
    }

    void Calibrator::addMeasurement(NsecTime timestamp) {
      lastTimestamp_ = timestamp;
      if (currentBatchStartTimestamp_ == -1)
        currentBatchStartTimestamp_ = timestamp;
      if (nsecToSec(timestamp - currentBatchStartTimestamp_) >=
          options_.windowDuration)
        addMeasurements();
    }

    void Calibrator::addMeasurements() {
      auto batch = boost::make_shared<OptimizationProblemSpline>();
      initSplines(options_.referenceSensor);
      batch->addSpline(translationSpline_, 0);
      batch->addSpline(rotationSpline_, 0);
      for (const auto& measurements : motionMeasurements_)
        addMotionErrorTerms(batch, measurements.first);
      clearMeasurements();
      currentBatchStartTimestamp_ = lastTimestamp_;
      batch->setGroupsOrdering({0, 1});
      if (options_.verbose) {
        std::cout << "calibration before batch: " << std::endl;
      }
      IncrementalEstimator::ReturnValue ret = estimator_->addBatch(batch);
      if (options_.verbose) {
        std::cout << "IG: " << ret.informationGain << std::endl;
        ret.batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
        std::cout << std::endl;
        std::cout << "calibration after batch: " << std::endl;
//        std::cout << *_odometryDesignVariables << std::endl;
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
      infoGainHistory_.push_back(ret.informationGain);
    }

    void Calibrator::clearMeasurements() {
      motionMeasurements_.clear();
    }

    void Calibrator::initSplines(size_t idx) {
      const auto& measurements = motionMeasurements_.at(idx);
      const auto numMeasurements = measurements.size();
      std::vector<NsecTime> timestamps;
      timestamps.reserve(numMeasurements);
      std::vector<Eigen::Vector3d> transPoses;
      transPoses.reserve(numMeasurements);
      std::vector<Eigen::Vector4d> rotPoses;
      rotPoses.reserve(numMeasurements);
      auto prevTransformation = sm::kinematics::Transformation();
      for (const auto& measurement : measurements) {
        const auto timestamp = measurement.first;
        const auto currentTransformation = prevTransformation *
          measurement.second.motion;
        prevTransformation = currentTransformation;
        transPoses.push_back(currentTransformation.t());
        auto q = currentTransformation.q();
        if (!rotPoses.empty())
          q = bestQuat(rotPoses.back(), q);
        rotPoses.push_back(q);
        timestamps.push_back(timestamp);
      }

      const auto elapsedTime = (timestamps.back() - timestamps.front()) /
        static_cast<double>(NsecTimePolicy::getOne());
      const auto measPerSec = std::lround(numMeasurements / elapsedTime);
      int numSegments;
      if (measPerSec > options_.splineKnotsPerSecond)
        numSegments = std::ceil(options_.splineKnotsPerSecond * elapsedTime);
      else
        numSegments = numMeasurements;

      translationSpline_ = boost::make_shared<TranslationSpline>(
        EuclideanBSpline<Eigen::Dynamic, 3, NsecTimePolicy>::CONF(
        EuclideanBSpline<Eigen::Dynamic, 3,
        NsecTimePolicy>::CONF::ManifoldConf(3), options_.transSplineOrder));
      BSplineFitter<TranslationSpline>::initUniformSpline(*translationSpline_,
        timestamps, transPoses, numSegments, options_.transSplineLambda);

      rotationSpline_ = boost::make_shared<RotationSpline>(
        UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>::CONF(
        UnitQuaternionBSpline<Eigen::Dynamic,
        NsecTimePolicy>::CONF::ManifoldConf(), options_.rotSplineOrder));
      BSplineFitter<RotationSpline>::initUniformSpline(*rotationSpline_,
        timestamps, rotPoses, numSegments, options_.rotSplineLambda);
    }

    void Calibrator::addMotionErrorTerms(const OptimizationProblemSplineSP&
        batch, size_t idx) {
      const auto& measurements = motionMeasurements_.at(idx);
      for (const auto& measurement : measurements) {
        const auto timestamp = measurement.first;
        auto translationExpressionFactory =
          translationSpline_->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          rotationSpline_->getExpressionFactoryAt<0>(timestamp);
//        ErrorTermPose::Input w_T_p;
//        w_T_p.head<3>() = it->second.w_r_wp;
//        w_T_p.tail<3>() = it->second.w_R_p;
//        ErrorTermPose::Covariance Q = ErrorTermPose::Covariance::Zero();
//        Q.topLeftCorner<3, 3>() = it->second.sigma2_w_r_wp;
//        Q.bottomRightCorner<3, 3>() = it->second.sigma2_w_R_p;

//        auto v_r_vp = EuclideanExpression(_odometryDesignVariables->v_r_vp);
//        auto w_R_v = Vector2RotationQuaternionExpressionAdapter::adapt(
//          rotationExpressionFactory.getValueExpression());
//        auto w_r_vp = w_R_v * v_r_vp;
//        auto w_r_wv = EuclideanExpression(
//          translationExpressionFactory.getValueExpression());
//        auto w_r_wp = w_r_wv + w_r_vp;
//        auto v_R_p = RotationExpression(_odometryDesignVariables->v_R_p);
//        auto w_R_p = w_R_v * v_R_p;
//        auto e_pose = boost::make_shared<ErrorTermPose>(
//          TransformationExpression(w_R_p, w_r_wp), w_T_p, Q);
//        batch->addErrorTerm(e_pose);
      }
    }

    void Calibrator::predictMotion(size_t idx) {
      const auto& measurements = motionMeasurements_.at(idx);
      for (const auto& measurement : measurements) {
        const auto timestamp = measurement.first;
        auto translationExpressionFactory =
          translationSpline_->getExpressionFactoryAt<0>(timestamp);
        auto rotationExpressionFactory =
          rotationSpline_->getExpressionFactoryAt<0>(timestamp);
      }
    }

  }
}
