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

#include <boost/shared_ptr.hpp>

namespace aslam {
  namespace calibration {

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
            _windowDuration(10.0) {
        }
        /// Window duration in seconds (-1 for standard batch mode)
        double _windowDuration;
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
      struct CANRearWheelsSpeedMeasurement {
        /// Measurement
        int16_t value;
      };
      /// Self type
      typedef CarCalibrator Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      CarCalibrator(Options = Options());
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
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Intrinsic CAN odometry design variables
      boost::shared_ptr<aslam::calibration::VectorDesignVariable<11> >
        _intrinsicCANOdometryDesignVariables;
      /// Intrinsic Applanix odometry design variables
      boost::shared_ptr<aslam::calibration::VectorDesignVariable<1> >
        _intrinsicApplanixOdometryDesignVariables;
      /// Extrinsic odometry center translation design variable
      boost::shared_ptr<aslam::backend::EuclideanPoint>
        _extrinsicOdometryTranslationDesignVariable;
      /// Extrinsic odometry center rotation design variable
      boost::shared_ptr<aslam::backend::RotationQuaternion>
        _extrinsicOdometryRotationDesignVariable;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_CALIBRATOR_H
