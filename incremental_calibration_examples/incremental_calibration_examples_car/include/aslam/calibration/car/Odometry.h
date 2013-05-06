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

/** \file Odometry.h
    \brief This file defines the Odometry class, which is the base class for
           odometry models.
  */

#ifndef ASLAM_CALIBRATION_ODOMETRY_H
#define ASLAM_CALIBRATION_ODOMETRY_H

#include <vector>

#include <Eigen/Core>

namespace aslam {
  namespace calibration {

    /** The class Odometry is the base class for odometry models.
        \brief Base odometry model
      */
    class Odometry {
    public:
      /** \name Types definitions
        @{
        */
      /// Vehicle parameters
      struct VehicleParameters {
        /// Radius of the rear left wheel [m]
        double _rearLeftWheelRadius;
        /// Radius of the rear right wheel [m]
        double _rearRightWheelRadius;
        /// Radius of the front left wheel [m]
        double _frontLeftWheelRadius;
        /// Radius of the front right wheel [m]
        double _frontRightWheelRadius;
        /// Wheel track [m]
        double _wheelTrack;
        /// Wheel base [m]
        double _wheelBase;
        /// Equality operator for comparing parameters
        bool operator == (const VehicleParameters &other) const {
          return (_rearLeftWheelRadius == other._rearLeftWheelRadius) &&
            (_rearRightWheelRadius == other._rearRightWheelRadius) &&
            (_frontLeftWheelRadius == other._frontLeftWheelRadius) &&
            (_frontRightWheelRadius == other._frontRightWheelRadius) &&
            (_wheelTrack == other._wheelTrack) &&
            (_wheelBase == other._wheelBase);
        }
      };
      /// Self type
      typedef Odometry Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      Odometry(const VehicleParameters& parameters, const Eigen::Vector3d&
        initialPose = Eigen::Vector3d::Zero());
      /// Copy constructor
      Odometry(const Self& other) = delete;
      /// Copy assignment operator
      Odometry& operator = (const Self& other) = delete;
      /// Move constructor
      Odometry(Self&& other) = delete;
      /// Move assignment operator
      Odometry& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~Odometry();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the vehicle parameters
      const VehicleParameters& getVehicleParameters() const;
      /// Returns the vehicle parameters
      VehicleParameters& getVehicleParameters();
      /// Returns the current pose
      const Eigen::Vector3d& getPose() const;
      /// Returns the pose history
      const std::vector<Eigen::Vector3d>& getPoseHistory() const;
      /// Resets pose history and sets initial pose
      void reset(const Eigen::Vector3d& initialPose = Eigen::Vector3d::Zero());
      /// Inserts a new pose
      void insertPose(const Eigen::Vector3d& pose);
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Updates odometry with COG displacement
      void updateCOGDisplacement(double dTrans, double dRot);
      /// Updates odometry with COG velocity
      void updateCOGVelocity(double vTrans, double vRot, double dT);
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
      /// Vehicle parameters
      VehicleParameters _parameters;
      /// Pose history
      std::vector<Eigen::Vector3d> _poses;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_ODOMETRY_H
