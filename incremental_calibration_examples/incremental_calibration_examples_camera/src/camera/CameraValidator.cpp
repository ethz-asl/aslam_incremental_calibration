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

#include "aslam/calibration/camera/CameraValidator.h"

#include <cmath>

#include <iostream>
#include <iterator>

#include <boost/make_shared.hpp>

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

#include <aslam/calibration/exceptions/BadArgumentException.h>
#include <aslam/calibration/base/Timestamp.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CameraValidator::CameraValidator(const sm::PropertyTree& intrinsics,
        const Options& options) :
        _options(options),
        _maxXError(0),
        _maxYError(0) {
      initVisionFramework(intrinsics);
    }

    CameraValidator::CameraValidator(const sm::PropertyTree& intrinsics, const
        sm::PropertyTree& config) :
        _maxXError(0),
        _maxYError(0) {
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
      _options.verbose = config.getBool("verbose", _options.verbose);

      // init vision framework
      initVisionFramework(intrinsics);
    }

    CameraValidator::~CameraValidator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const CameraValidator::Options& CameraValidator::getOptions() const {
      return _options;
    }

    CameraValidator::Options& CameraValidator::getOptions() {
      return _options;
    }

    const std::vector<CameraValidator::ObservationPtr>&
        CameraValidator::getObservations() const {
      return _observations;
    }

    Eigen::VectorXd CameraValidator::getReprojectionErrorMean() const {
      if (_reprojectionErrorsStatistics.getValid())
        return _reprojectionErrorsStatistics.getDistribution().getMean();
      else
        return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd CameraValidator::getReprojectionErrorVariance() const {
      if (_reprojectionErrorsStatistics.getValid())
        return _reprojectionErrorsStatistics.getDistribution().getCovariance().
          diagonal();
      else
        return Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd CameraValidator::getReprojectionErrorStandardDeviation()
        const {
      if (_reprojectionErrorsStatistics.getValid())
        return getReprojectionErrorVariance().array().sqrt();
      else
        return Eigen::VectorXd::Zero(0);
    }

    double CameraValidator::getReprojectionErrorMaxXError() const {
      return _maxXError;
    }

    double CameraValidator::getReprojectionErrorMaxYError() const {
      return _maxYError;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void CameraValidator::initVisionFramework(const sm::PropertyTree&
        intrinsics) {
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
      if (intrinsics.getString("projection/type") == "omni")
        _geometry = boost::make_shared<DistortedOmniCameraGeometry>(intrinsics);
      else if (intrinsics.getString("projection/type") == "pinhole")
        _geometry = boost::make_shared<DistortedPinholeCameraGeometry>(
          intrinsics);
      else
        throw BadArgumentException<std::string>(
          intrinsics.getString("projection/type"),
          "unkown camera projection type", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);

      // create detector
      _detector = boost::make_shared<Detector>(_geometry, _calibrationTarget,
        _options.useAdaptiveThreshold, _options.normalizeImage,
        _options.filterQuads, _options.doSubpixelRefinement);
    }

    bool CameraValidator::addImage(const cv::Mat& image, sm::timing::NsecTime
        timestamp) {
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

      // transformation from camera to target
      auto T_t_c = observation->T_t_c();

      // transformation from target to camera
      auto T_c_t = T_t_c.inverse();

      // iterate over checkerboard corners
      for (size_t i = 0; i < _calibrationTarget->size(); ++i) {
        auto targetPoint = sm::kinematics::toHomogeneous(
          _calibrationTarget->point(i));
        Eigen::Vector2d observedPoint;
        bool success = observation->imagePoint(i, observedPoint);
        if (!success)
          continue;
        Eigen::VectorXd predictedPoint;
        success = _geometry->vsHomogeneousToKeypoint(T_c_t * targetPoint,
          predictedPoint);
        if (!success)
          continue;
        Eigen::Vector2d error = predictedPoint - observedPoint;
        _reprojectionErrorsStatistics.addPoint(error);
        if (std::fabs(error(0)) > _maxXError)
          _maxXError = std::fabs(error(0));
        if (std::fabs(error(1)) > _maxYError)
          _maxYError = std::fabs(error(1));
      }

      // store observation for later use if needed
      _observations.push_back(observation);

      return true;
    }

  }
}
