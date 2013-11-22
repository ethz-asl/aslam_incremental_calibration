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

/** \file CameraCalibrator.h
    \brief This file defines the CameraCalibrator class which implements
           the camera calibration algorithm.
  */

#ifndef ASLAM_CALIBRATION_CAMERA_CALIBRATOR_H
#define ASLAM_CALIBRATION_CAMERA_CALIBRATOR_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace cv {

  class Mat;

}
namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace cameras {

    class GridDetector;
    class GridCalibrationTarget;
    class CameraGeometryBase;
    template<typename PROJECTION_T, typename SHUTTER_T, typename MASK_T>
      class CameraGeometry;
    template<typename DISTORTION_T> class OmniProjection;
    template<typename DISTORTION_T> class PinholeProjection;
    class RadialTangentialDistortion;
    class GlobalShutter;
    class NoMask;

  }
  namespace backend {

    class HomogeneousPoint;

  }
  namespace calibration {

    class IncrementalEstimator;

    /** The class CameraCalibrator implements the camera calibration algorithm.
        \brief Camera calibration algorithm.
      */
    class CameraCalibrator {
    public:
      /** \name Types definitions
        @{
        */
      /// Incremental estimator shared pointer type
      typedef boost::shared_ptr<IncrementalEstimator> IncrementalEstimatorPtr;
      /// Calibration target type
      typedef aslam::cameras::GridCalibrationTarget CalibrationTarget;
      /// Calibration target shared pointer type
      typedef boost::shared_ptr<CalibrationTarget> CalibrationTargetPtr;
      /// Detector type
      typedef aslam::cameras::GridDetector Detector;
      /// Detector shared pointer type
      typedef boost::shared_ptr<Detector> DetectorPtr;
      /// Camera geometry type
      typedef aslam::cameras::CameraGeometryBase CameraGeometry;
      /// Camera geometry shared pointer type
      typedef boost::shared_ptr<CameraGeometry> CameraGeometryPtr;
      /// Distorted omni camera geometry type
      typedef aslam::cameras::CameraGeometry<aslam::cameras::OmniProjection<
          aslam::cameras::RadialTangentialDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>
          DistortedOmniCameraGeometry;
      /// Distorted pinhole camera geometry type
      typedef aslam::cameras::CameraGeometry<aslam::cameras::PinholeProjection<
          aslam::cameras::RadialTangentialDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>
          DistortedPinholeCameraGeometry;
      /// Landmark design variable type
      typedef aslam::backend::HomogeneousPoint LandmarkDesignVariable;
      /// Landmark design variable shared pointer type
      typedef boost::shared_ptr<LandmarkDesignVariable>
        LandmarkDesignVariablePtr;
      /// Self type
      typedef CameraCalibrator Self;
      /// Options for the camera calibrator
      struct Options {
        /// Number of rows in the checkerboard
        size_t rows;
        /// Number of columns in the checkerboard
        size_t cols;
        /// Spacing between two consecutive rows in meters
        double rowSpacingMeters;
        /// Spacing between two consecutive columns in meters
        double colSpacingMeters;
        /// Detector type
        std::string detectorType;
        /// Use adaptive threshold for OpenCV detector
        bool useAdaptiveThreshold;
        /// Use normalizeImage for OpenCV detector
        bool normalizeImage;
        /// Use filterQuads for OpenCV detector
        bool filterQuads;
        /// Use doSubpixelRefinement for OpenCV detector
        bool doSubpixelRefinement;
        /// Camera projection type
        std::string cameraProjectionType;
        /// Estimate the landmarks
        bool estimateLandmarks;
        /// Verbose mode
        bool verbose;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CameraCalibrator();
      /// Constructs calibrator with configuration in property tree
      CameraCalibrator(const sm::PropertyTree& config);
      /// Copy constructor
      CameraCalibrator(const Self& other) = delete;
      /// Copy assignment operator
      CameraCalibrator& operator = (const Self& other) = delete;
      /// Move constructor
      CameraCalibrator(Self&& other) = delete;
      /// Move assignment operator
      CameraCalibrator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~CameraCalibrator();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the current options
      const Options& getOptions() const;
      /// Returns the current options
      Options& getOptions();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Init geometry from an image
      bool initGeometry(const cv::Mat& image);
      /// Add an image to the calibrator
      void addImage(const cv::Mat& image, sm::timing::NsecTime timestamp);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Incremental estimator
      IncrementalEstimatorPtr _estimator;
      /// Calibration target
      CalibrationTargetPtr _calibrationTarget;
      /// Detector
      DetectorPtr _detector;
      /// Camera geometry
      CameraGeometryPtr _geometry;
      /// Landmark design variables
      std::vector<LandmarkDesignVariablePtr> _landmarkDesignVariables;
      /// Geometry initialized properly
      bool _geometryInitialized;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAMERA_CALIBRATOR_H
