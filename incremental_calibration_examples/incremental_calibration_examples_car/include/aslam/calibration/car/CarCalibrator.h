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

/** \file CarCalibrator.h
    \brief This file defines the CarCalibrator class which implements
           the car calibration algorithm.
  */

#ifndef ASLAM_CALIBRATION_CAR_CALIBRATOR_H
#define ASLAM_CALIBRATION_CAR_CALIBRATOR_H

#include <utility>
#include <vector>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

#include "aslam/calibration/car/MeasurementsContainer.h"

namespace bsplines {

  struct NsecTimePolicy;

}
namespace aslam {
  namespace backend {

    class EuclideanPoint;
    class RotationQuaternion;
    class EuclideanExpression;

  }
  namespace calibration {

    template <int M> class VectorDesignVariable;
    class OptimizationProblemSpline;
    class IncrementalEstimator;
    struct ApplanixNavigationMeasurement;
    struct WheelsSpeedMeasurement;
    struct SteeringMeasurement;
    struct ApplanixDMIMeasurement;

    /** The class CarCalibrator implements the car calibration algorithm.
        \brief Car calibration algorithm.
      */
    class CarCalibrator {
    public:
      /** \name Types definitions
        @{
        */
      /// Options for the car calibrator
      struct Options {
        Options() :
            windowDuration(10.0),
            transSplineLambda(0),
            rotSplineLambda(0),
            knotsPerSecond(5),
            transSplineOrder(4),
            rotSplineOrder(4),
            linearVelocityTolerance(1e-1),
            dmiCovariance((Eigen::Matrix<double, 1, 1>() << 10).finished()),
            fwsCovariance((Eigen::Matrix2d() << 500, 0, 0, 500).finished()),
            rwsCovariance((Eigen::Matrix2d() << 500, 0, 0, 500).finished()),
            steeringCovariance((Eigen::Matrix<double, 1, 1>()
              << 10).finished()),
            wheelSpeedSensorCutoff(350),
            verbose(true) {
        }
        /// Window duration in seconds
        double windowDuration;
        /// Translation spline lambda
        double transSplineLambda;
        /// Rotation spline lambda
        double rotSplineLambda;
        /// Pose measurements per second desired for the spline
        int knotsPerSecond;
        /// Translation spline order
        int transSplineOrder;
        /// Rotation spline order
        int rotSplineOrder;
        /// Tolerance for rejecting low speed measurements
        double linearVelocityTolerance;
        /// Covariance for DMI measurements
        Eigen::Matrix<double, 1, 1> dmiCovariance;
        /// Covariance for front wheels speed measurements
        Eigen::Matrix2d fwsCovariance;
        /// Covariance for rear wheels speed measurements
        Eigen::Matrix2d rwsCovariance;
        /// Covariance for steering measurements
        Eigen::Matrix<double, 1, 1> steeringCovariance;
        /// Wheel speed sensor cutoff
        uint16_t wheelSpeedSensorCutoff;
        /// Verbose option
        bool verbose;
      };
      /// Shared pointer to incremental estimator
      typedef boost::shared_ptr<IncrementalEstimator> IncrementalEstimatorSP;
      /// Shared pointer to euclidean point
      typedef boost::shared_ptr<aslam::backend::EuclideanPoint>
        EuclideanPointSP;
      /// Shared pointer to rotation quaternion
      typedef boost::shared_ptr<aslam::backend::RotationQuaternion>
        RotationQuaternionSP;
      /// Shared pointer to odometry intrinsic design variable
      typedef boost::shared_ptr<VectorDesignVariable<12> > OdoDesignVariableSP;
      /// Calibration design variables
      struct CalibrationDesignVariables {
        /// Intrinsic odometry design variable
        OdoDesignVariableSP intrinsicOdoDesignVariable;
        /// Extrinsic odometry center translation design variable
        EuclideanPointSP extrinsicOdoTranslationDesignVariable;
        /// Extrinsic odometry center rotation design variable
        RotationQuaternionSP extrinsicOdoRotationDesignVariable;
      };
      /// Rotation spline
      typedef typename aslam::splines::OPTBSpline<typename
        bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Translation spline
      typedef typename aslam::splines::OPTBSpline<typename
        bsplines::EuclideanBSpline<Eigen::Dynamic, 3,
        bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;
      /// Euclidean spline shared pointer
      typedef boost::shared_ptr<TranslationSpline> TranslationSplineSP;
      /// Optimization problem shared pointer
      typedef boost::shared_ptr<OptimizationProblemSpline>
        OptimizationProblemSplineSP;
      /// Applanix navigation measurements
      typedef MeasurementsContainer<ApplanixNavigationMeasurement>::Type
        ApplanixNavigationMeasurements;
      /// Applanix DMI measurements
      typedef MeasurementsContainer<ApplanixDMIMeasurement>::Type
        ApplanixDMIMeasurements;
      /// Wheel speeds measurements
      typedef MeasurementsContainer<WheelsSpeedMeasurement>::Type
        WheelsSpeedMeasurements;
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
      /// Constructor
      CarCalibrator(const IncrementalEstimatorSP& estimator, const
        CalibrationDesignVariables& calibrationDesignVariables, const Options&
        options = Options());
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
      /// Returns the calibration design variables
      const CalibrationDesignVariables& getCalibrationDesignVariables() const;
      /// Returns the calibration design variables
      CalibrationDesignVariables& getCalibrationDesignVariables();
      /// Returns the incremental estimator
      const IncrementalEstimatorSP getEstimator() const;
      /// Returns the incremental estimator
      IncrementalEstimatorSP getEstimator();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds an Applanix POS LV navigation measurement
      void addNavigationMeasurement(const ApplanixNavigationMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds an Applanix POS LV encoder measurement
      void addDMIMeasurement(const ApplanixDMIMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a CAN front wheels speed measurement
      void addFrontWheelsMeasurement(const WheelsSpeedMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a CAN rear wheels speed measurement
      void addRearWheelsMeasurement(const WheelsSpeedMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds a CAN steering measurement
      void addSteeringMeasurement(const SteeringMeasurement& data,
        sm::timing::NsecTime timestamp);
      /// Adds the currently stored measurements to the estimator
      void addMeasurements();
      /// Clears the stored measurements
      void clearMeasurements();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Adds a new measurement
      void addMeasurement(sm::timing::NsecTime timestamp);
      /// Adds Applanix navigation error terms
      void addNavigationErrorTerms(const ApplanixNavigationMeasurements&
        measurements, const OptimizationProblemSplineSP& batch);
      /// Adds Applanix encoders error terms
      void addDMIErrorTerms(const ApplanixDMIMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Adds CAN front wheels speed error terms
      void addFrontWheelsErrorTerms(const WheelsSpeedMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Adds CAN rear wheels speed error terms
      void addRearWheelsErrorTerms(const WheelsSpeedMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Adds CAN steering error terms
      void addSteeringErrorTerms(const SteeringMeasurements& measurements,
        const OptimizationProblemSplineSP& batch);
      /// Adds vehicle model error terms
      void addVehicleErrorTerms(const OptimizationProblemSplineSP& batch);
      /// Returns linear and angular velocity in odometry frame from B-spline
      std::pair<aslam::backend::EuclideanExpression,
        aslam::backend::EuclideanExpression> getOdometryVelocities(
        sm::timing::NsecTime timestamp, const TranslationSplineSP&
        translationSpline, const RotationSplineSP& rotationSpline) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Calibration design variables
      CalibrationDesignVariables _calibrationDesignVariables;
      /// Incremental estimator
      IncrementalEstimatorSP _estimator;
      /// Current batch starting timestamp
      sm::timing::NsecTime _currentBatchStartTimestamp;
      /// Last timestamp
      sm::timing::NsecTime _lastTimestamp;
      /// Stored Applanix navigation measurements
      ApplanixNavigationMeasurements _navigationMeasurements;
      /// Stored Applanix encoder measurements
      ApplanixDMIMeasurements _dmiMeasurements;
      /// Stored CAN front wheels speed measurements
      WheelsSpeedMeasurements _frontWheelsSpeedMeasurements;
      /// Stored CAN rear wheels speed measurements
      WheelsSpeedMeasurements _rearWheelsSpeedMeasurements;
      /// Stored CAN steering measurements
      SteeringMeasurements _steeringMeasurements;
      /// Current rotation spline
      RotationSplineSP _rotationSpline;
      /// Current translation spline
      TranslationSplineSP _translationSpline;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_CALIBRATOR_H
