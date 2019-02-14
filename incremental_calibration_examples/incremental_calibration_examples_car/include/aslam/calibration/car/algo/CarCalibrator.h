/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file CarCalibrator.h
    \brief This file defines the CarCalibrator class which implements
           the car calibration algorithm.
  */

#ifndef ASLAM_CALIBRATION_CAR_CALIBRATOR_H
#define ASLAM_CALIBRATION_CAR_CALIBRATOR_H

#include <vector>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

#include "aslam/calibration/car/data/MeasurementsContainer.h"
#include "aslam/calibration/car/algo/CarCalibratorOptions.h"

namespace sm {

  class PropertyTree;

}
namespace bsplines {

  struct NsecTimePolicy;

}
namespace aslam {
  namespace calibration {

    class OptimizationProblemSpline;
    class IncrementalEstimator;
    struct PoseMeasurement;
    struct VelocitiesMeasurement;
    struct WheelSpeedsMeasurement;
    struct SteeringMeasurement;
    struct DMIMeasurement;
    struct OdometryDesignVariables;

    /** The class CarCalibrator implements the car calibration algorithm.
        \brief Car calibration algorithm.
      */
    class CarCalibrator {
    public:
      /** \name Types definitions
        @{
        */
      /// Options for the car calibrator
      typedef CarCalibratorOptions Options;
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
      /// Velocities measurements
      typedef MeasurementsContainer<VelocitiesMeasurement>::Type
        VelocitiesMeasurements;
      /// Applanix DMI measurements
      typedef MeasurementsContainer<DMIMeasurement>::Type DMIMeasurements;
      /// Wheel speeds measurements
      typedef MeasurementsContainer<WheelSpeedsMeasurement>::Type
        WheelSpeedsMeasurements;
      /// Steering measurements
      typedef MeasurementsContainer<SteeringMeasurement>::Type
        SteeringMeasurements;
      /// Self type
      typedef CarCalibrator Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs calibrator with configuration in property tree
      CarCalibrator(const sm::PropertyTree& config);
      /// Copy constructor
      CarCalibrator(const Self& other) = delete;
      /// Copy assignment operator
      CarCalibrator& operator = (const Self& other) = delete;
      /// Move constructor
      CarCalibrator(Self&& other) = delete;
      /// Move assignment operator
      CarCalibrator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~CarCalibrator();
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
      /// Returns the velocities measurements
      const VelocitiesMeasurements& getVelocitiesMeasurements() const;
      /// Returns the velocities predictions
      const VelocitiesMeasurements& getVelocitiesPredictions() const;
      /// Returns the velocities predictions errors
      const std::vector<Eigen::VectorXd>& getVelocitiesPredictionErrors() const;
      /// Returns the velocities predictions squared errors
      const std::vector<double>& getVelocitiesPredictionErrors2() const;
      /// Returns the DMI measurements
      const DMIMeasurements& getDMIMeasurements() const;
      /// Returns the DMI predictions
      const DMIMeasurements& getDMIPredictions() const;
      /// Returns the DMI predictions errors
      const std::vector<Eigen::VectorXd>& getDMIPredictionErrors() const;
      /// Returns the DMI predictions squared errors
      const std::vector<double>& getDMIPredictionErrors2() const;
      /// Returns the rear wheels measurements
      const WheelSpeedsMeasurements& getRearWheelsMeasurements() const;
      /// Returns the rear wheels predictions
      const WheelSpeedsMeasurements& getRearWheelsPredictions() const;
      /// Returns the rear wheels prediction errors
      const std::vector<Eigen::VectorXd>& getRearWheelsPredictionErrors() const;
      /// Returns the rear wheels prediction squared errors
      const std::vector<double>& getRearWheelsPredictionErrors2() const;
      /// Returns the front wheels measurements
      const WheelSpeedsMeasurements& getFrontWheelsMeasurements() const;
      /// Returns the front wheels predictions
      const WheelSpeedsMeasurements& getFrontWheelsPredictions() const;
      /// Returns the front wheels prediction errors
      const std::vector<Eigen::VectorXd>& getFrontWheelsPredictionErrors()
        const;
      /// Returns the front wheels prediction squared errors
      const std::vector<double>& getFrontWheelsPredictionErrors2() const;
      /// Returns the steering measurements
      const SteeringMeasurements& getSteeringMeasurements() const;
      /// Returns the steering predictions
      const SteeringMeasurements& getSteeringPredictions() const;
      /// Returns the steering prediction errors
      const std::vector<Eigen::VectorXd>& getSteeringPredictionErrors() const;
      /// Returns the steering prediction squared errors
      const std::vector<double>& getSteeringPredictionErrors2() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds a pose measurement
      void addPoseMeasurement(const PoseMeasurement& pose, sm::timing::NsecTime
        timestamp);
      /// Adds a velocities measurement
      void addVelocitiesMeasurement(const VelocitiesMeasurement& vel,
        sm::timing::NsecTime timestamp);
      /// Adds an Applanix POS LV encoder measurement
      void addDMIMeasurement(const DMIMeasurement& data, sm::timing::NsecTime
        timestamp);
      /// Adds a CAN front wheels speed measurement
      void addFrontWheelsMeasurement(const WheelSpeedsMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a CAN rear wheels speed measurement
      void addRearWheelsMeasurement(const WheelSpeedsMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a CAN steering measurement
      void addSteeringMeasurement(const SteeringMeasurement& data,
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
      /// Adds velocities error terms
      void addVelocitiesErrorTerms(const VelocitiesMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts velocities measurements
      void predictVelocities(const VelocitiesMeasurements& measurements);
      /// Adds Applanix encoders error terms
      void addDMIErrorTerms(const DMIMeasurements& measurements, const
        OptimizationProblemSplineSP& batch);
      /// Predicts DMI measurements
      void predictDMI(const DMIMeasurements& measurements);
      /// Adds CAN front wheels speed error terms
      void addFrontWheelsErrorTerms(const WheelSpeedsMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts CAN data fw measurements
      void predictFrontWheels(const WheelSpeedsMeasurements& measurements);
      /// Adds CAN rear wheels speed error terms
      void addRearWheelsErrorTerms(const WheelSpeedsMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts CAN data rw measurements
      void predictRearWheels(const WheelSpeedsMeasurements& measurements);
      /// Adds CAN steering error terms
      void addSteeringErrorTerms(const SteeringMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Predicts CAN data st measurements
      void predictSteering(const SteeringMeasurements& measurements);
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
      /// Stored velocities measurements
      VelocitiesMeasurements _velocitiesMeasurements;
      /// Predicted velocities measurements
      VelocitiesMeasurements _velocitiesMeasurementsPred;
      /// Velocities measurements prediction errors
      std::vector<Eigen::VectorXd> _velocitiesMeasurementsPredErrors;
      /// Velocities measurements squared errors
      std::vector<double> _velocitiesMeasurementsPredErrors2;
      /// Stored Applanix encoder measurements
      DMIMeasurements _dmiMeasurements;
      /// Predicted Applanix encoder measurements
      DMIMeasurements _dmiMeasurementsPred;
      /// DMI measurements prediction errors
      std::vector<Eigen::VectorXd> _dmiMeasurementsPredErrors;
      /// DMI measurements squared errors
      std::vector<double> _dmiMeasurementsPredErrors2;
      /// Stored CAN front wheels speed measurements
      WheelSpeedsMeasurements _frontWheelSpeedsMeasurements;
      /// Predicted CAN front wheels speed measurements
      WheelSpeedsMeasurements _frontWheelSpeedsMeasurementsPred;
      /// Front wheels measurements errors
      std::vector<Eigen::VectorXd> _frontWheelSpeedsMeasurementsPredErrors;
      /// Front wheels measurements squared errors
      std::vector<double> _frontWheelSpeedsMeasurementsPredErrors2;
      /// Stored CAN rear wheels speed measurements
      WheelSpeedsMeasurements _rearWheelSpeedsMeasurements;
      /// Predicted CAN rear wheels speed measurements
      WheelSpeedsMeasurements _rearWheelSpeedsMeasurementsPred;
      /// Rear wheels measurements errors
      std::vector<Eigen::VectorXd> _rearWheelSpeedsMeasurementsPredErrors;
      /// Rear wheels measurements squared errors
      std::vector<double> _rearWheelSpeedsMeasurementsPredErrors2;
      /// Stored CAN steering measurements
      SteeringMeasurements _steeringMeasurements;
      /// Predicted CAN steering measurements
      SteeringMeasurements _steeringMeasurementsPred;
      /// Steering measurements errors
      std::vector<Eigen::VectorXd> _steeringMeasurementsPredErrors;
      /// Steering measurements squared errors
      std::vector<double> _steeringMeasurementsPredErrors2;
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

#endif // ASLAM_CALIBRATION_CAR_CALIBRATOR_H
