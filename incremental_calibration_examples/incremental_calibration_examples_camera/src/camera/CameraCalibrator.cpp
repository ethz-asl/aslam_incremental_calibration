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

#include <iostream>
#include <iterator>

#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <sm/PropertyTree.hpp>

#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/Transformation.hpp>

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
#include <aslam/ReprojectionError.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/exceptions/BadArgumentException.h>
#include <aslam/calibration/exceptions/InvalidOperationException.h>

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
          "unkown calibration target type", __FILE__, __LINE__,
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
    }

    bool CameraCalibrator::initGeometry(const cv::Mat& image) {
      if (_detector->initCameraGeometryFromObservation(image)) {
        _geometryInitialized = true;
        initBatch();
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
      if (_options.cameraProjectionType == "omni") {
        auto dv = boost::make_shared<aslam::backend::CameraDesignVariable<
          DistortedOmniCameraGeometry> >(boost::dynamic_pointer_cast<
          DistortedOmniCameraGeometry>(_geometry));
        dv->setActive(true, true, false);
        _batch->addDesignVariable(dv->projectionDesignVariable(),
          _options.calibrationGroupId);
        _batch->addDesignVariable(dv->distortionDesignVariable(),
          _options.calibrationGroupId);
        _batch->addDesignVariable(dv->shutterDesignVariable(),
          _options.calibrationGroupId);
      }
      else {
        auto dv = boost::make_shared<aslam::backend::CameraDesignVariable<
          DistortedPinholeCameraGeometry> >(boost::dynamic_pointer_cast<
          DistortedPinholeCameraGeometry>(_geometry));
        dv->setActive(true, true, false);
        _batch->addDesignVariable(dv->projectionDesignVariable(),
          _options.calibrationGroupId);
        _batch->addDesignVariable(dv->distortionDesignVariable(),
          _options.calibrationGroupId);
        _batch->addDesignVariable(dv->shutterDesignVariable(),
          _options.calibrationGroupId);
      }
    }

    void CameraCalibrator::addObservation(const Observation& observation) {
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
            _geometry.get());
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
      aslam::cameras::GridCalibrationTargetObservation observation;
      const bool status = _detector->findTarget(image, aslam::Time(
        sm::timing::nsecToSec(timestamp)), observation);
      if (!status) {
        std::cerr << __PRETTY_FUNCTION__ << ": target not found at time "
          << sm::timing::nsecToSec(timestamp) << std::endl;
        return status;
      }

      // add observation to the batch
      addObservation(observation);

      // add batch if needed
      if (_batchNumImages == _options.batchNumImages) {
        _estimator->addBatch(_batch);
        initBatch();
        _batchNumImages = 0;
      }

      return true;
    }

  }
}
