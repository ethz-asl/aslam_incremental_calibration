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

#include <boost/shared_ptr.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>

namespace aslam {
  namespace backend {

    class EuclideanPoint;
    class RotationQuaternion;
    class EuclideanExpression;

  }
  namespace splines {

    class BSplinePoseDesignVariable;

  }
  namespace calibration {

    template <int M> class VectorDesignVariable;
    class IncrementalEstimator;

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
            poseSplineOrder(4),
            poseSplineLambda(1e-1),
            poseMeasPerSecDesired(5) {
        }
        /// Window duration in seconds
        double windowDuration;
        /// Pose spline order
        int poseSplineOrder;
        /// Pose spline lambda
        double poseSplineLambda;
        /// Pose measurements per second desired
        int poseMeasPerSecDesired;
      };
      /// Applanix vehicle navigation measurement in local ENU system
      struct ApplanixNavigationMeasurement {
        /// x pose [m]
        double x;
        /// y pose [m]
        double y;
        /// z pose [m]
        double z;
        /// roll [rad]
        double roll;
        /// pitch [rad]
        double pitch;
        /// yaw [rad]
        double yaw;
        /// body linear velocity in x in world frame [m/s]
        double v_x;
        /// body linear velocity in y in world frame [m/s]
        double v_y;
        /// body linear velocity in z in world frame [m/s]
        double v_z;
        /// body angular velocity in x in body frame [rad/s]
        double om_x;
        /// body angular velocity in y in body frame [rad/s]
        double om_y;
        /// body angular velocity in z in body frame [rad/s]
        double om_z;
        /// body linear acceleration in x in body frame [m/s^2]
        double a_x;
        /// body linear acceleration in y in body frame [m/s^2]
        double a_y;
        /// body linear acceleration in z in body frame [m/s^2]
        double a_z;
        /// linear velocity [m/s]
        double v;
        /// x pose sigma^2
        double x_sigma2;
        /// y pose sigma^2
        double y_sigma2;
        /// z pose sigma^2
        double z_sigma2;
        /// roll sigma^2
        double roll_sigma2;
        /// pitch sigma^2
        double pitch_sigma2;
        /// yaw sigma^2
        double yaw_sigma2;
        /// linear velocity in x sigma^2
        double v_x_sigma2;
        /// linear velocity in y sigma^2
        double v_y_sigma2;
        /// linear velocity in z sigma^2
        double v_z_sigma2;
      };
      /// Applanix encoder measurement
      struct ApplanixEncoderMeasurement {
        /// Signed distance travelled
        double signedDistanceTraveled;
        /// Unsigned distance travelled
        double unsignedDistanceTravelled;
      };
      /// CAN front wheels speed measurement
      struct CANFrontWheelsSpeedMeasurement {
        /// Left speed measurement
        uint16_t left;
        /// Right speed measurement
        uint16_t right;
      };
      /// CAN rear wheels speed measurement
      struct CANRearWheelsSpeedMeasurement {
        /// Left speed measurement
        uint16_t left;
        /// Right speed measurement
        uint16_t right;
      };
      /// CAN steering measurement
      struct CANSteeringMeasurement {
        /// Measurement
        int16_t value;
      };
      /// Shared pointer to incremental estimator
      typedef boost::shared_ptr<IncrementalEstimator> IncrementalEstimatorSP;
      /// Shared pointer to euclidean point
      typedef boost::shared_ptr<aslam::backend::EuclideanPoint>
        EuclideanPointSP;
      /// Shared pointer to rotation quaternion
      typedef boost::shared_ptr<aslam::backend::RotationQuaternion>
        RotationQuaternionSP;
      /// Shared pointer to CAN intrinsic design variable
      typedef boost::shared_ptr<VectorDesignVariable<11> > CANDesignVariableSP;
      /// Shared pointer to Applanix DMI intrinsic design variable
      typedef boost::shared_ptr<VectorDesignVariable<1> > DMIDesignVariableSP;
      /// Calibration design variables
      struct CalibrationDesignVariables {
        /// Intrinsic CAN odometry design variable
        CANDesignVariableSP intrinsicCANDesignVariable;
        /// Intrinsic Applanix DMI odometry design variable
        DMIDesignVariableSP intrinsicDMIDesignVariable;
        /// Extrinsic odometry center translation design variable
        EuclideanPointSP extrinsicOdometryTranslationDesignVariable;
        /// Extrinsic odometry center rotation design variable
        RotationQuaternionSP extrinsicOdometryRotationDesignVariable;
      };
      /// Applanix vehicle navigation measurement container
      typedef std::vector<std::pair<double, ApplanixNavigationMeasurement> >
        ApplanixNavigationMeasurements;
      /// Applanix encoder measurement container
      typedef std::vector<std::pair<double, ApplanixEncoderMeasurement> >
        ApplanixEncoderMeasurements;
      /// CAN front wheels speed measurement container
      typedef std::vector<std::pair<double, CANFrontWheelsSpeedMeasurement> >
        CANFrontWheelsSpeedMeasurements;
      /// CAN rear wheels speed measurement container
      typedef std::vector<std::pair<double, CANRearWheelsSpeedMeasurement> >
        CANRearWheelsSpeedMeasurements;
      /// CAN steering measurement container
      typedef std::vector<std::pair<double, CANSteeringMeasurement> >
        CANSteeringMeasurements;
      /// Shared pointer to B-spline pose design variable
      typedef boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable>
        BSplinePoseDesignVariableSP;
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
      void addMeasurement(const ApplanixNavigationMeasurement& data,
        double timestamp);
      /// Adds an Applanix POS LV encoder measurement
      void addMeasurement(const ApplanixEncoderMeasurement& data,
        double timestamp);
      /// Adds a CAN front wheels speed measurement
      void addMeasurement(const CANFrontWheelsSpeedMeasurement& data,
        double timestamp);
      /// Adds a CAN rear wheels speed measurement
      void addMeasurement(const CANRearWheelsSpeedMeasurement& data,
        double timestamp);
      /// Adds a CAN steering measurement
      void addMeasurement(const CANSteeringMeasurement& data, double timestamp);
      /// Adds the currently stored measurements to the estimator
      void addMeasurements();
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Adds a new measurement
      void addMeasurement(double timestamp);
      /// Adds Applanix navigation error terms
      void addErrorTerms(const ApplanixNavigationMeasurements& measurements,
        IncrementalEstimator::BatchSP batch);
      /// Adds Applanix encoders error terms
      void addErrorTerms(const ApplanixEncoderMeasurements& measurements,
        IncrementalEstimator::BatchSP batch);
      /// Adds CAN front wheels speed error terms
      void addErrorTerms(const CANFrontWheelsSpeedMeasurements& measurements,
        IncrementalEstimator::BatchSP batch);
      /// Adds CAN rear wheels speed error terms
      void addErrorTerms(const CANRearWheelsSpeedMeasurements& measurements,
        IncrementalEstimator::BatchSP batch);
      /// Adds CAN steering error terms
      void addErrorTerms(const CANSteeringMeasurements& measurements,
        IncrementalEstimator::BatchSP batch);
      /// Returns linear and angular velocity in odometry frame
      std::pair<aslam::backend::EuclideanExpression,
        aslam::backend::EuclideanExpression> getOdometryVelocities(double
        timestamp, const BSplinePoseDesignVariableSP& bspdv) const;
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
      double _currentBatchStartTimestamp;
      /// Last timestamp
      double _lastTimestamp;
      /// Stored Applanix navigation measurements
      ApplanixNavigationMeasurements _applanixNavigationMeasurements;
      /// Stored Applanix encoder measurements
      ApplanixEncoderMeasurements _applanixEncoderMeasurements;
      /// Stored CAN front wheels speed measurements
      CANFrontWheelsSpeedMeasurements _canFrontWheelsSpeedMeasurements;
      /// Stored CAN rear wheels speed measurements
      CANRearWheelsSpeedMeasurements _canRearWheelsSpeedMeasurements;
      /// Stored CAN steering measurements
      CANSteeringMeasurements _canSteeringMeasurements;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_CALIBRATOR_H
