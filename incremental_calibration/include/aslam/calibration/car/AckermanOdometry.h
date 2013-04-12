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

/** \file AckermanOdometry.h
    \brief This file defines the AckermanOdometry class, which implements
           an odometry model based on Ackerman geometry.
  */

#ifndef ASLAM_CALIBRATION_ACKERMAN_ODOMETRY_H
#define ASLAM_CALIBRATION_ACKERMAN_ODOMETRY_H

#include "aslam/calibration/car/Odometry.h"

namespace aslam {
  namespace calibration {

    /** The class AckermanOdometry implements an odometry model based on
        Ackerman geometry.
        \brief Ackerman odometry
      */
    class AckermanOdometry :
      public Odometry {
    public:
      /** \name Types definitions
        @{
        */
      /// Self type
      typedef AckermanOdometry Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      AckermanOdometry(const Odometry::VehicleParameters& parameters,
        const Eigen::Vector3d& initialPose);
      /// Copy constructor
      AckermanOdometry(const Self& other) = delete;
      /// Copy assignment operator
      AckermanOdometry& operator = (const Self& other) = delete;
      /// Move constructor
      AckermanOdometry(Self&& other) = delete;
      /// Move assignment operator
      AckermanOdometry& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~AckermanOdometry();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Updates odometry with wheel translational velocities and steering
      void updateWheelTranslationalVelocitiesSteering(double vRearLeftWheel,
        double vRearRightWheel, double vFrontLeftWheel, double vFrontRightWheel,
        double steering, double dT);
      /// Update odometry with wheel rotational velocities and steering
      void updateWheelRotationalVelocitiesSteering(double wRearLeftWheel,
        double wRearRightWheel, double wFrontLeftWheel, double wFrontRightWheel,
        double steering, double dT);
      /// Update odometry with wheel displacements and steering
      void updateWheelDisplacementsSteering(double dRearLeftWheel,
        double dRearRightWheel, double dFrontLeftWheel, double dFrontRightWheel,
        double steering);
      /** @}
        */

    protected:

    };

  }
}

#endif // ASLAM_CALIBRATION_ACKERMAN_ODOMETRY_H
