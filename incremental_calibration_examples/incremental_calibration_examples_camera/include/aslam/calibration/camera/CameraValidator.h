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

/** \file CameraValidator.h
    \brief This file defines the CameraValidator class which implements
           the camera validation algorithm.
  */

#ifndef ASLAM_CALIBRATION_CAMERA_VALIDATOR_H
#define ASLAM_CALIBRATION_CAMERA_VALIDATOR_H

#include <cstddef>

#include <string>
#include <vector>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/calibration/statistics/EstimatorML.h>
#include <aslam/calibration/statistics/NormalDistribution.h>

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
    class GridCalibrationTargetObservation;
    class CameraGeometryBase;
    template<typename PROJECTION_T, typename SHUTTER_T, typename MASK_T>
      class CameraGeometry;
    template<typename DISTORTION_T> class OmniProjection;
    template<typename DISTORTION_T> class PinholeProjection;
    class RadialTangentialDistortion;
    class GlobalShutter;
    class NoMask;

  }
  namespace calibration {

    /** The class CameraValidator implements the camera validation algorithm.
        \brief Camera validation algorithm.
      */
    class CameraValidator {
    public:
      /** \name Types definitions
        @{
        */
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
      /// Grid observation
      typedef aslam::cameras::GridCalibrationTargetObservation Observation;
      /// Grid observation shared pointer
      typedef boost::shared_ptr<Observation> ObservationPtr;
      /// Self type
      typedef CameraValidator Self;
      /// Options for the camera calibrator
      struct Options {
        /// Default constructor
        Options() :
            rows(6),
            cols(7),
            rowSpacingMeters(0.03),
            colSpacingMeters(0.03),
            detectorType("checkerboard"),
            useAdaptiveThreshold(true),
            normalizeImage(false),
            filterQuads(false),
            doSubpixelRefinement(true),
            verbose(false) {}
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
        /// Verbose mode
        bool verbose;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CameraValidator(const sm::PropertyTree& intrinsics, const Options&
        options = Options());
      /// Constructs validator with configuration in property tree
      CameraValidator(const sm::PropertyTree& intrinsics, const
        sm::PropertyTree& config);
      /// Copy constructor
      CameraValidator(const Self& other) = delete;
      /// Copy assignment operator
      CameraValidator& operator = (const Self& other) = delete;
      /// Move constructor
      CameraValidator(Self&& other) = delete;
      /// Move assignment operator
      CameraValidator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~CameraValidator();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the current options
      const Options& getOptions() const;
      /// Returns the current options
      Options& getOptions();
      /// Returns the current observations
      const std::vector<ObservationPtr>& getObservations() const;
      /// Returns the reprojection error mean
      Eigen::VectorXd getReprojectionErrorMean() const;
      /// Returns the reprojection error variance
      Eigen::VectorXd getReprojectionErrorVariance() const;
      /// Returns the reprojection error standard deviation
      Eigen::VectorXd getReprojectionErrorStandardDeviation() const;
      /// Returns the reprojection error maximum x error
      double getReprojectionErrorMaxXError() const;
      /// Returns the reprojection error maximum y error
      double getReprojectionErrorMaxYError() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Add an image to the validator
      bool addImage(const cv::Mat& image, sm::timing::NsecTime timestamp);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Init the vision framework
      void initVisionFramework(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Calibration target
      CalibrationTargetPtr _calibrationTarget;
      /// Detector
      DetectorPtr _detector;
      /// Camera geometry
      CameraGeometryPtr _geometry;
      /// Observations
      std::vector<ObservationPtr> _observations;
      /// Reprojection error statistics
      EstimatorML<NormalDistribution<2> > _reprojectionErrorsStatistics;
      /// Maximum reprojection error in x
      double _maxXError;
      /// Maximum reprojection error in y
      double _maxYError;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAMERA_VALIDATOR_H
