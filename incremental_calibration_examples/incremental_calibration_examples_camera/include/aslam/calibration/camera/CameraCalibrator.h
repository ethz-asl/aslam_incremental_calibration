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

#include <cstddef>

#include <string>
#include <vector>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace cv {

  class Mat;

}
namespace sm {

  class PropertyTree;

}
namespace aslam {

  class CameraGeometryDesignVariableContainer;

  namespace cameras {

    class GridDetector;
    class GridCalibrationTarget;
    class GridCalibrationTargetObservation;
    class CameraGeometryBase;

  }
  namespace backend {

    class HomogeneousPoint;

  }
  namespace calibration {

    class IncrementalEstimator;
    class OptimizationProblem;

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
      /// Landmark design variable type
      typedef aslam::backend::HomogeneousPoint LandmarkDesignVariable;
      /// Landmark design variable shared pointer type
      typedef boost::shared_ptr<LandmarkDesignVariable>
        LandmarkDesignVariablePtr;
      /// Batch for the estimator
      typedef boost::shared_ptr<OptimizationProblem> BatchPtr;
      /// Grid observation
      typedef aslam::cameras::GridCalibrationTargetObservation Observation;
      /// Grid observation shared pointer
      typedef boost::shared_ptr<Observation> ObservationPtr;
      /// Camera intrinsics design variable container
      typedef aslam::CameraGeometryDesignVariableContainer
        CameraDesignVariableContainer;
      /// Camera intrinsics design variable containter shared pointer
      typedef boost::shared_ptr<CameraDesignVariableContainer>
        CameraDesignVariableContainerPtr;
      /// Self type
      typedef CameraCalibrator Self;
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
            cameraProjectionType("pinhole"),
            estimateLandmarks(false),
            landmarksGroupId(2),
            calibrationGroupId(0),
            transformationsGroupId(1),
            batchNumImages(1),
            useMEstimator(false),
            sigma2(1.0),
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
        /// Camera projection type
        std::string cameraProjectionType;
        /// Estimate the landmarks
        bool estimateLandmarks;
        /// Landmark design variables group ID
        size_t landmarksGroupId;
        /// Calibration design variables group ID
        size_t calibrationGroupId;
        /// Transformation design variables group ID
        size_t transformationsGroupId;
        /// Number of images in a batch
        size_t batchNumImages;
        /// Use M-Estimator
        bool useMEstimator;
        /// Variance of the measurements (assume isotropic Gaussian)
        double sigma2;
        /// Verbose mode
        bool verbose;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor with estimator and options
      CameraCalibrator(const IncrementalEstimatorPtr& estimator, const Options&
        options = Options());
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
      /// Returns the incremental estimator
      const IncrementalEstimatorPtr getEstimator() const;
      /// Returns the incremental estimator
      IncrementalEstimatorPtr getEstimator();
      /// Returns the number of images in the current batch
      size_t getBatchNumImages() const;
      /// Returns the current projection parameters
      Eigen::VectorXd getProjection() const;
      /// Returns the current projection variance
      Eigen::VectorXd getProjectionVariance() const;
      /// Returns the current projection standard deviation
      Eigen::VectorXd getProjectionStandardDeviation() const;
      /// Returns the current distortion parameters
      Eigen::VectorXd getDistortion() const;
      /// Returns the current distortion variance
      Eigen::VectorXd getDistortionVariance() const;
      /// Returns the current distortion standard deviation
      Eigen::VectorXd getDistortionStandardDeviation() const;
      /// Returns the current batch observations
      const std::vector<ObservationPtr>& getBatchObservations() const;
      /// Returns the current estimator observations
      const std::vector<ObservationPtr>& getEstimatorObservations() const;
      /// Returns the transformation matrix for an observation
      Eigen::Matrix4d getTransformation(size_t idx) const;
      /// Returns the current initial cost for the estimator
      double getInitialCost() const;
      /// Returns the current final cost for the estimator
      double getFinalCost() const;
      /// Returns the current nullspace
      Eigen::MatrixXd getNullSpace(bool scaled = false) const;
      /// Returns the statistics for the reprojection error
      void getStatistics(Eigen::VectorXd&
        mean, Eigen::VectorXd& variance, Eigen::VectorXd& standardDeviation,
        double& maxXError, double& maxYError, size_t& numOutliers);
      /// Returns the last checkerboard image
      void getLastCheckerboardImage(cv::Mat& image) const;
      /// Returns the errors and the squared mahalanobis distances
      void getErrors(std::vector<Eigen::Vector2d>& errors, std::vector<double>&
        errorsMd2);
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Init geometry from an image
      bool initGeometry(const cv::Mat& image);
      /// Add an image to the calibrator
      bool addImage(const cv::Mat& image, sm::timing::NsecTime timestamp);
      /// Process the current batch
      void processBatch();
      /// Write camera parameters to property tree
      void write(sm::PropertyTree& config) const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Init the vision framework
      void initVisionFramework();
      /// Init batch
      void initBatch();
      /// Add an observation into the batch
      void addObservation(const Observation& observation);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Incremental estimator
      IncrementalEstimatorPtr _estimator;
      /// Estimator batch
      BatchPtr _batch;
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
      /// Number of images in the batch
      size_t _batchNumImages;
      /// Camera intrinsics design variable container
      CameraDesignVariableContainerPtr _cameraDesignVariableContainer;
      /// Observations in the current batch
      std::vector<ObservationPtr> _batchObservations;
      /// Observations accepted by the estimator
      std::vector<ObservationPtr> _estimatorObservations;
      /// Last observation
      ObservationPtr _lastObservation;
      /// Quantile for outlier detection
      double _q;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAMERA_CALIBRATOR_H
