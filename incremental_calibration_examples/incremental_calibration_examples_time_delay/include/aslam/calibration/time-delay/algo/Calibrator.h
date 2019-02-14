/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Calibrator.h
    \brief This file defines the Calibrator class which implements the
           calibration algorithm.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_H
#define ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_H

#include <vector>
#include <cstdint>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

#include "aslam/calibration/time-delay/data/MeasurementsContainer.h"
#include "aslam/calibration/time-delay/algo/CalibratorOptions.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    class OptimizationProblemSpline;
    class IncrementalEstimator;
    struct PoseMeasurement;
    struct WheelSpeedMeasurement;
    struct OdometryDesignVariables;

    /** The class Calibrator implements the calibration algorithm.
        \brief Calibration algorithm.
      */
    class Calibrator {
    public:
      /** \name Types definitions
        @{
        */
      /// Options for the calibrator
      typedef CalibratorOptions Options;
      /// Shared pointer to incremental estimator
      typedef boost::shared_ptr<IncrementalEstimator> IncrementalEstimatorSP;
      /// Odometry design variables shared pointer
      typedef boost::shared_ptr<OdometryDesignVariables>
        OdometryDesignVariablesSP;
      /// Rotation spline
      typedef aslam::splines::OPTBSpline<
        bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Translation spline
      typedef aslam::splines::OPTBSpline<
        bsplines::EuclideanBSpline<Eigen::Dynamic, 3,
        bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;
      /// Euclidean spline shared pointer
      typedef boost::shared_ptr<TranslationSpline> TranslationSplineSP;
      /// Optimization problem shared pointer
      typedef boost::shared_ptr<OptimizationProblemSpline>
        OptimizationProblemSplineSP;
      /// Pose measurements
      typedef MeasurementsContainer<PoseMeasurement>::Type PoseMeasurements;
      /// Wheel speed measurements
      typedef MeasurementsContainer<WheelSpeedMeasurement>::Type
        WheelSpeedMeasurements;
      /// Self type
      typedef Calibrator Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs calibrator with configuration in property tree
      Calibrator(const sm::PropertyTree& config);
      /// Copy constructor
      Calibrator(const Self& other) = delete;
      /// Copy assignment operator
      Calibrator& operator = (const Self& other) = delete;
      /// Move constructor
      Calibrator(Self&& other) = delete;
      /// Move assignment operator
      Calibrator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~Calibrator();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the current options
      const Options& getOptions() const;
      /// Returns the current options
      Options& getOptions();
      /// Returns the odometry calibration design variables
      const OdometryDesignVariablesSP& getOdometryDesignVariables() const;
      /// Returns the odometry calibration design variables
      OdometryDesignVariablesSP& getOdometryDesignVariables();
      /// Returns the incremental estimator
      const IncrementalEstimatorSP getEstimator() const;
      /// Returns the incremental estimator
      IncrementalEstimatorSP getEstimator();
      /// Unprocessed measurements in the pipeline?
      bool unprocessedMeasurements() const;
      /// Returns the information gain history
      const std::vector<double> getInformationGainHistory() const;
      /// Returns the calibration variables history
      const std::vector<Eigen::VectorXd> getOdometryVariablesHistory() const;
      /// Returns the variance of the estimates
      Eigen::VectorXd getOdometryVariablesVariance() const;
      /// Returns the current translation spline
      const TranslationSplineSP& getTranslationSpline() const;
      /// Returns the current rotation spline
      const RotationSplineSP& getRotationSpline() const;
      /// Returns the pose measurements
      const PoseMeasurements& getPoseMeasurements() const;
      /// Returns the pose predictions
      const PoseMeasurements& getPosePredictions() const;
      /// Returns the pose predictions errors
      const std::vector<Eigen::VectorXd>& getPosePredictionErrors() const;
      /// Returns the pose predictions squared errors
      const std::vector<double>& getPosePredictionErrors2() const;
      /// Returns the left wheel measurements
      const WheelSpeedMeasurements& getLeftWheelMeasurements() const;
      /// Returns the left wheel predictions
      const WheelSpeedMeasurements& getLeftWheelPredictions() const;
      /// Returns the left wheel prediction errors
      const std::vector<Eigen::VectorXd>& getLeftWheelPredictionErrors() const;
      /// Returns the left wheel prediction squared errors
      const std::vector<double>& getLeftWheelPredictionErrors2() const;
      /// Returns the right wheel measurements
      const WheelSpeedMeasurements& getRightWheelMeasurements() const;
      /// Returns the right wheel predictions
      const WheelSpeedMeasurements& getRightWheelPredictions() const;
      /// Returns the right wheel prediction errors
      const std::vector<Eigen::VectorXd>& getRightWheelPredictionErrors() const;
      /// Returns the right wheel prediction squared errors
      const std::vector<double>& getRightWheelPredictionErrors2() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds a pose measurement
      void addPoseMeasurement(const PoseMeasurement& posel, sm::timing::NsecTime
        timestamp);
      /// Adds a left wheel speed measurement
      void addLeftWheelMeasurement(const WheelSpeedMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a right wheel speed measurement
      void addRightWheelMeasurement(const WheelSpeedMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds the currently stored measurements to the estimator
      void addMeasurements();
      /// Clears the stored measurements
      void clearMeasurements();
      /// Predicts the stored measurements
      void predict();
      /// Clears the predictions
      void clearPredictions();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Adds a new measurement
      void addMeasurement(sm::timing::NsecTime timestamp);
      /// Adds pose error terms
      void addPoseErrorTerms(const PoseMeasurements& measurements, const
        OptimizationProblemSplineSP& batch);
      /// Predicts pose measurements
      void predictPoses(const PoseMeasurements& measurements);
      /// Adds left wheel speed error terms
      void addLeftWheelErrorTerms(const WheelSpeedMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts left wheel measurements
      void predictLeftWheel(const WheelSpeedMeasurements& measurements);
      /// Adds right wheel speed error terms
      void addRightWheelErrorTerms(const WheelSpeedMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts right wheel measurements
      void predictRightWheel(const WheelSpeedMeasurements& measurements);
      /// Initializes the splines from a batch of pose measurements
      void initSplines(const PoseMeasurements& measurements);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Odoemtry calibration design variables
      OdometryDesignVariablesSP _odometryDesignVariables;
      /// Incremental estimator
      IncrementalEstimatorSP _estimator;
      /// Current batch starting timestamp
      sm::timing::NsecTime _currentBatchStartTimestamp;
      /// Last timestamp
      sm::timing::NsecTime _lastTimestamp;
      /// Stored pose measurements
      PoseMeasurements _poseMeasurements;
      /// Predicted pose measurements
      PoseMeasurements _poseMeasurementsPred;
      /// Pose measurements errors
      std::vector<Eigen::VectorXd> _poseMeasurementsPredErrors;
      /// Pose measurements squared errors
      std::vector<double> _poseMeasurementsPredErrors2;
      /// Stored left wheel speed measurements
      WheelSpeedMeasurements _leftWheelSpeedMeasurements;
      /// Predicted left wheel speed measurements
      WheelSpeedMeasurements _leftWheelSpeedMeasurementsPred;
      /// Left wheel measurements errors
      std::vector<Eigen::VectorXd> _leftWheelSpeedMeasurementsPredErrors;
      /// Left wheel measurements squared errors
      std::vector<double> _leftWheelSpeedMeasurementsPredErrors2;
      /// Stored right wheel speed measurements
      WheelSpeedMeasurements _rightWheelSpeedMeasurements;
      /// Predicted right wheel speed measurements
      WheelSpeedMeasurements _rightWheelSpeedMeasurementsPred;
      /// Right wheel measurements errors
      std::vector<Eigen::VectorXd> _rightWheelSpeedMeasurementsPredErrors;
      /// Right wheel measurements squared errors
      std::vector<double> _rightWheelSpeedMeasurementsPredErrors2;
      /// Current rotation spline
      RotationSplineSP _rotationSpline;
      /// Current translation spline
      TranslationSplineSP _translationSpline;
      /// Information gain history
      std::vector<double> _infoGainHistory;
      /// Calibration variables history
      std::vector<Eigen::VectorXd> _odometryVariablesHistory;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_H
