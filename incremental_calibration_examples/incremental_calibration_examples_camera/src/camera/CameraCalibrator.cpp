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

#include <boost/make_shared.hpp>

#include <sm/PropertyTree.hpp>

#include <sm/kinematics/homogeneous_coordinates.hpp>

#include <aslam/cameras/GridCalibrationTarget.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>

#include <aslam/backend/HomogeneousPoint.hpp>

#include <aslam/calibration/exceptions/BadArgumentException.h>
#include <aslam/calibration/exceptions/InvalidOperationException.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CameraCalibrator::CameraCalibrator() :
        _geometryInitialized(false) {
    }

    CameraCalibrator::CameraCalibrator(const sm::PropertyTree& config) :
        _geometryInitialized(false) {
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

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    bool CameraCalibrator::initGeometry(const cv::Mat& image) {
      if (_detector->initCameraGeometryFromObservation(image)) {
        _geometryInitialized = true;
        return true;
      }
      else
        return false;
    }

    void CameraCalibrator::addImage(const cv::Mat& image, sm::timing::NsecTime
        timestamp) {
      if (!_geometryInitialized)
        throw InvalidOperationException("geometry not initialized", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);

    }

  }
}
