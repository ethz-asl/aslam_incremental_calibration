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

#include "aslam/calibration/camera/CameraCalibrator.h"

#include <cmath>

#include <iostream>
#include <iterator>

#include <boost/make_shared.hpp>

#include <sm/PropertyTree.hpp>

#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <sm/boost/null_deleter.hpp>

#include <aslam/Time.hpp>

#include <aslam/cameras/GridCalibrationTarget.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include <aslam/ReprojectionError.hpp>
#include <aslam/CameraGeometryDesignVariableContainer.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/exceptions/BadArgumentException.h>
#include <aslam/calibration/exceptions/InvalidOperationException.h>
#include <aslam/calibration/exceptions/OutOfBoundException.h>
#include <aslam/calibration/base/Timestamp.h>
#include <aslam/calibration/statistics/EstimatorML.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CameraCalibrator::CameraCalibrator(const IncrementalEstimatorPtr& estimator,
        const Options& options) :
        _options(options),
        _estimator(estimator),
        _geometryInitialized(false),
        _batchNumImages(0) {
      initVisionFramework();
    }

    CameraCalibrator::CameraCalibrator(const sm::PropertyTree& config) :
        _geometryInitialized(false),
        _batchNumImages(0) {
      // read the options from the property tree
      _options.rows = config.getInt("rows", _options.rows);
      _options.cols = config.getInt("cols", _options.cols);
      _options.rowSpacingMeters = config.getDouble("rowSpacingMeters",
        _options.rowSpacingMeters);
      _options.colSpacingMeters = config.getDouble("colSpacingMeters",
        _options.colSpacingMeters);
      _options.detectorType = config.getString("detectorType",
        _options.detectorType);
      _options.useAdaptiveThreshold = config.getBool("useAdaptiveThreshold",
        _options.useAdaptiveThreshold);
      _options.normalizeImage = config.getBool("normalizeImage",
        _options.normalizeImage);
      _options.filterQuads = config.getBool("filterQuads",
        _options.filterQuads);
      _options.doSubpixelRefinement = config.getBool("doSubpixelRefinement",
        _options.doSubpixelRefinement);
      _options.cameraProjectionType = config.getString("cameraProjectionType",
        _options.cameraProjectionType);
      _options.estimateLandmarks = config.getBool("estimateLandmarks",
        _options.estimateLandmarks);
      _options.landmarksGroupId = config.getInt("landmarksGroupId",
        _options.landmarksGroupId);
      _options.calibrationGroupId = config.getInt("calibrationGroupId",
        _options.calibrationGroupId);
      _options.transformationsGroupId = config.getInt("transformationsGroupId",
        _options.transformationsGroupId);
      _options.batchNumImages = config.getInt("batchNumImages",
        _options.batchNumImages);
      _options.verbose = config.getBool("verbose", _options.verbose);

      // init vision framework
      initVisionFramework();

      // create the underlying estimator
      _estimator = boost::make_shared<IncrementalEstimator>(
        sm::PropertyTree(config, "estimator"));
    }

    CameraCalibrator::~CameraCalibrator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const CameraCalibrator::Options& CameraCalibrator::getOptions() const {
      return _options;
    }

    CameraCalibrator::Options& CameraCalibrator::getOptions() {
      return _options;
    }

    const CameraCalibrator::IncrementalEstimatorPtr
        CameraCalibrator::getEstimator() const {
      return _estimator;
    }

    CameraCalibrator::IncrementalEstimatorPtr CameraCalibrator::getEstimator() {
      return _estimator;
    }

    size_t CameraCalibrator::getBatchNumImages() const {
      return _batchNumImages;
    }

    Eigen::VectorXd CameraCalibrator::getProjection() const {
      Eigen::MatrixXd params;
      _geometry->getParameters(params, true, false, false);
      return params;
    }

    Eigen::VectorXd CameraCalibrator::getProjectionVariance() const {
      if (_estimator->getNumBatches())
        return _estimator->getMarginalizedCovariance().diagonal().head(
          _geometry->minimalDimensionsProjection());
      else
        return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd CameraCalibrator::getProjectionStandardDeviation() const {
      if (_estimator->getNumBatches())
        return getProjectionVariance().array().sqrt();
      else
        return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd CameraCalibrator::getDistortion() const {
      Eigen::MatrixXd params;
      _geometry->getParameters(params, false, true, false);
      return params;
    }

    Eigen::VectorXd CameraCalibrator::getDistortionVariance() const {
      if (_estimator->getNumBatches())
        return _estimator->getMarginalizedCovariance().diagonal().tail(
          _geometry->minimalDimensionsDistortion());
      else
        return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd CameraCalibrator::getDistortionStandardDeviation() const {
      if (_estimator->getNumBatches())
        return getDistortionVariance().array().sqrt();
      else
        return Eigen::VectorXd::Zero(0);
    }

    const std::vector<CameraCalibrator::ObservationPtr>&
        CameraCalibrator::getBatchObservations() const {
      return _batchObservations;
    }

    const std::vector<CameraCalibrator::ObservationPtr>&
        CameraCalibrator::getEstimatorObservations() const {
      return _estimatorObservations;
    }

    double CameraCalibrator::getInitialCost() const {
      return _estimator->getInitialCost();
    }

    double CameraCalibrator::getFinalCost() const {
      return _estimator->getFinalCost();
    }

    Eigen::Matrix4d CameraCalibrator::getTransformation(size_t idx) const {
      if (idx >= _estimatorObservations.size())
        throw OutOfBoundException<size_t>(idx, _estimatorObservations.size(),
          "idx must be stricly smaller than the number of observations",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      auto dvs = _estimator->getProblem()->getDesignVariablesGroup(
        _options.transformationsGroupId);
      auto q_dv = const_cast<aslam::backend::RotationQuaternion*>(
        dynamic_cast<const aslam::backend::RotationQuaternion*>(
        dvs.at(idx * 2)));
      auto t_dv = dynamic_cast<const aslam::backend::EuclideanPoint*>(
        dvs.at(idx * 2 + 1));
      sm::kinematics::Transformation T(q_dv->getQuaternion(),
        t_dv->toEuclidean());
      return T.T();
    }

    Eigen::MatrixXd CameraCalibrator::getNullSpace() const {
      return _estimator->getMarginalizedNullSpace();
    }

    void CameraCalibrator::getReprojectionErrorStatistics(Eigen::VectorXd&
        mean, Eigen::VectorXd& variance, Eigen::VectorXd& standardDeviation,
        double& maxXError, double& maxYError) {
      auto problem = const_cast<IncrementalOptimizationProblem*>(
        _estimator->getProblem());
      EstimatorML<NormalDistribution<2> > reprojectionErrorsStatistics;
      maxXError = 0;
      maxYError = 0;
      for (size_t i = 0; i != problem->numErrorTerms(); ++i) {
        auto et = problem->errorTerm(i);
        et->evaluateError();
        auto error = dynamic_cast<aslam::ReprojectionError*>(et)->error();
        reprojectionErrorsStatistics.addPoint(error);
        if (std::fabs(error(0)) > maxXError)
          maxXError = std::fabs(error(0));
        if (std::fabs(error(1)) > maxYError)
          maxYError = std::fabs(error(1));
      }
      if (reprojectionErrorsStatistics.getValid()) {
        mean = reprojectionErrorsStatistics.getDistribution().getMean();
        variance = reprojectionErrorsStatistics.getDistribution().
          getCovariance().diagonal();
        standardDeviation = variance.array().sqrt();
      }
      else {
        mean.resize(0);
        variance.resize(0);
        standardDeviation.resize(0);
        maxXError = 0;
        maxYError = 0;
      }
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CameraCalibrator::initVisionFramework() {
      // create calibration target
      CalibrationTarget::DetectorType detectorType;
      if (_options.detectorType == "checkerboard")
        detectorType = CalibrationTarget::DetectorType::Checkerboard;
      else if (_options.detectorType == "circleGrid")
        detectorType = CalibrationTarget::DetectorType::CircleGrid;
      else if (_options.detectorType == "aprilGrid")
        detectorType = CalibrationTarget::DetectorType::AprilGrid;
      else
        throw BadArgumentException<std::string>(_options.detectorType,
          "unkown calibration target type", __FILE__,__LINE__,
          __PRETTY_FUNCTION__);
      _calibrationTarget = boost::make_shared<CalibrationTarget>(_options.rows,
        _options.cols, _options.rowSpacingMeters, _options.colSpacingMeters,
        detectorType);

      // create camera geometry
      if (_options.cameraProjectionType == "omni")
        _geometry = boost::make_shared<DistortedOmniCameraGeometry>();
      else if (_options.cameraProjectionType == "pinhole")
        _geometry = boost::make_shared<DistortedPinholeCameraGeometry>();
      else
        throw BadArgumentException<std::string>(_options.cameraProjectionType,
          "unkown camera projection type", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);

      // create detector
      _detector = boost::make_shared<Detector>(_geometry, _calibrationTarget,
        _options.useAdaptiveThreshold, _options.normalizeImage,
        _options.filterQuads, _options.doSubpixelRefinement);

      // create design variables for landmarks
      _landmarkDesignVariables.reserve(_calibrationTarget->size());
      for (size_t i = 0; i < _calibrationTarget->size(); ++i) {
        auto dv =  boost::make_shared<LandmarkDesignVariable>(
          sm::kinematics::toHomogeneous(_calibrationTarget->point(i)));
        dv->setActive(_options.estimateLandmarks);
        _landmarkDesignVariables.push_back(dv);
      }

      // create camera intrinsics design variable
      _cameraDesignVariableContainer = boost::make_shared<
        CameraDesignVariableContainer>(_geometry, true, true, false);
    }

    bool CameraCalibrator::initGeometry(const cv::Mat& image) {
      if (_geometryInitialized)
        return true;
      if (_detector->initCameraGeometryFromObservation(image)) {
        _geometryInitialized = true;
        return true;
      }
      else
        return false;
    }

    void CameraCalibrator::initBatch() {
      // create batch / overwrite older if already existing
      _batch = boost::make_shared<OptimizationProblem>();

      // add the landmark design variables
      for (auto it = _landmarkDesignVariables.cbegin();
          it != _landmarkDesignVariables.cend(); ++it)
        _batch->addDesignVariable(*it, _options.landmarksGroupId);

      // add the calibration design variables
      aslam::backend::DesignVariable::set_t cameraDesignVariables;
      _cameraDesignVariableContainer->getDesignVariables(cameraDesignVariables);
      for (auto it = cameraDesignVariables.begin();
          it != cameraDesignVariables.end(); ++it)
        _batch->addDesignVariable(
          boost::shared_ptr<aslam::backend::DesignVariable>(
          *it, sm::null_deleter()), _options.calibrationGroupId);

      // clear the currently stored observations
      _batchObservations.clear();
    }

    void CameraCalibrator::addObservation(const Observation& observation) {
      // if the batch does not exist, create it
      if (!_batch)
        initBatch();

      // get the transformation that takes points from camera coordinates to
      // target coordinates
      auto T_t_c = const_cast<Observation&>(observation).T_t_c();

      // create a rotation quaternion design variable and add it
      auto q_dv = boost::make_shared<aslam::backend::RotationQuaternion>(
        T_t_c.q());
      q_dv->setActive(true);
      _batch->addDesignVariable(q_dv, _options.transformationsGroupId);

      // create an Euclidean point design variable and add it
      auto t_dv = boost::make_shared<aslam::backend::EuclideanPoint>(T_t_c.t());
      t_dv->setActive(true);
      _batch->addDesignVariable(t_dv, _options.transformationsGroupId);

      // expression for the transformation
      aslam::backend::RotationExpression q_dv_e(q_dv);
      aslam::backend::EuclideanExpression t_dv_e(t_dv);
      aslam::backend::TransformationExpression T_t_c_e(q_dv_e, t_dv_e);

      // inverse transformation
      auto T_c_t_e = T_t_c_e.inverse();

      // add the reprojection error terms
      for (auto it = _landmarkDesignVariables.cbegin();
          it != _landmarkDesignVariables.cend(); ++it) {
        auto targetPoint = (*it)->toExpression();
        Eigen::Vector2d obsPoint;
        const bool success = observation.imagePoint(std::distance(
          _landmarkDesignVariables.cbegin(), it), obsPoint);
        if (success) {
          auto re = boost::make_shared<aslam::ReprojectionError>(obsPoint,
            Eigen::Matrix2d::Identity(), T_c_t_e * targetPoint,
            _cameraDesignVariableContainer.get());
          _batch->addErrorTerm(re);
        }
      }

      _batchNumImages++;
    }

    bool CameraCalibrator::addImage(const cv::Mat& image, sm::timing::NsecTime
        timestamp) {
      if (!_geometryInitialized)
        throw InvalidOperationException("geometry not initialized", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);

      // find the target in the input image
      auto observation = boost::make_shared<Observation>();
      const bool status = _detector->findTarget(image, aslam::Time(
        sm::timing::nsecToSec(timestamp)), *observation);
      if (!status) {
        if (_options.verbose)
          std::cerr << __PRETTY_FUNCTION__ << ": target not found at time "
            << sm::timing::nsecToSec(timestamp) << std::endl;
        return status;
      }
      else {
        if (_options.verbose)
          std::cout << __PRETTY_FUNCTION__ << ": target found at time "
            << sm::timing::nsecToSec(timestamp) << std::endl;
      }

      // add observation to the batch
      addObservation(*observation);
      _batchObservations.push_back(observation);

      // add batch if needed
      if (_batchNumImages == _options.batchNumImages)
        processBatch();

      return true;
    }

    void CameraCalibrator::processBatch() {
      if (!_batch)
        return;
      auto ret = _estimator->addBatch(_batch);
      if (ret.batchAccepted) {
        _estimatorObservations.insert(_estimatorObservations.begin(),
          _batchObservations.begin(), _batchObservations.end());
      }
      if (_options.verbose) {
        std::cout << std::endl;
        ret.batchAccepted ? std::cout << "ACCEPTED" : std::cout << "REJECTED";
        std::cout << std::endl;
        std::cout << "MI: " << ret.mutualInformation << std::endl;
        std::cout << "null space: " << std::endl << ret.nullSpace << std::endl;
        std::cout << "projection: " << getProjection().transpose() << std::endl;
        std::cout << "projection standard deviation: "
          << getProjectionStandardDeviation().transpose() << std::endl;
        std::cout << "distortion: " << getDistortion().transpose() << std::endl;
        std::cout << "distortion standard deviation: "
          << getDistortionStandardDeviation().transpose() << std::endl;
        Eigen::VectorXd mean, variance, standardDeviation;
        double maxXError, maxYError;
        getReprojectionErrorStatistics(mean, variance, standardDeviation,
          maxXError, maxYError);
        std::cout << "reprojection error mean: " << mean.transpose()
          << std::endl;
        std::cout << "reprojection error standard deviation: " <<
          standardDeviation.transpose() << std::endl;
        std::cout << "max x reprojection error: " << maxXError << std::endl;
        std::cout << "max y reprojection error: " << maxYError << std::endl;
        std::cout << "number of images used: " << _estimatorObservations.size()
          << std::endl;
         std::cout << std::endl;
      }
      initBatch();
      _batchNumImages = 0;
    }

    void CameraCalibrator::write(sm::PropertyTree& config) const {
      auto projection = getProjection();
      auto distortion = getDistortion();
      if (_options.cameraProjectionType == "omni") {
        config.setDouble("projection/xi", projection(0));
        config.setDouble("projection/fu", projection(1));
        config.setDouble("projection/fv", projection(2));
        config.setDouble("projection/cu", projection(3));
        config.setDouble("projection/cv", projection(4));
        config.setInt("projection/ru",
          dynamic_cast<DistortedOmniCameraGeometry*>(
          _geometry.get())->projection().ru());
        config.setInt("projection/rv",
          dynamic_cast<DistortedOmniCameraGeometry*>(
          _geometry.get())->projection().rv());
      }
      else {
        config.setDouble("projection/fu", projection(0));
        config.setDouble("projection/fv", projection(1));
        config.setDouble("projection/cu", projection(2));
        config.setDouble("projection/cv", projection(3));
        config.setInt("projection/ru",
          dynamic_cast<DistortedPinholeCameraGeometry*>
          (_geometry.get())->projection().ru());
        config.setInt("projection/rv",
          dynamic_cast<DistortedPinholeCameraGeometry*>(
          _geometry.get())->projection().rv());
      }
      config.setString("projection/type", _options.cameraProjectionType);
      config.setDouble("projection/distortion/k1", distortion(0));
      config.setDouble("projection/distortion/k2", distortion(1));
      config.setDouble("projection/distortion/p1", distortion(2));
      config.setDouble("projection/distortion/p2", distortion(3));
      config.setDouble("shutter/line-delay", 0);
      config.setString("mask/mask-file", "");
    }

  }
}
