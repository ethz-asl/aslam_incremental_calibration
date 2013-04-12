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

/** \file DifferentialOdometry.h
    \brief This file defines the DifferentialOdometry class, which implements
           an odometry model based on differential geometry.
  */

#ifndef ASLAM_CALIBRATION_DIFFERENTIAL_ODOMETRY_H
#define ASLAM_CALIBRATION_DIFFERENTIAL_ODOMETRY_H

#include "aslam/calibration/car/Odometry.h"

namespace aslam {
  namespace calibration {

    /** The class DifferentialOdometry implements an odometry model based on
        differential geometry.
        \brief Differential odometry
      */
    class DifferentialOdometry :
      public Odometry {
    public:
      /** \name Types definitions
        @{
        */
      /// Self type
      typedef DifferentialOdometry Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor with initial pose
      DifferentialOdometry(const Odometry::VehicleParameters& parameters,
        const Eigen::Vector3d& initialPose = Eigen::Vector3d::Zero());
      /// Copy constructor
      DifferentialOdometry(const Self& other) = delete;
      /// Copy assignment operator
      DifferentialOdometry& operator = (const Self& other) = delete;
      /// Move constructor
      DifferentialOdometry(Self&& other) = delete;
      /// Move assignment operator
      DifferentialOdometry& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~DifferentialOdometry();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Updates odometry with wheel translational velocities
      void updateWheelTranslationalVelocities(double vLeftWheel,
        double vRightWheel, double dT);
      /// Update odometry with wheel rotational velocities
      void updateWheelRotationalVelocities(double wLeftWheel,
        double wRightWheel, double dT);
      /// Update odometry with wheel displacements
      void updateWheelDisplacements(double dLeftWheel, double dRightWheel);
      /** @}
        */

    protected:

    };

  }
}

#endif // ASLAM_CALIBRATION_DIFFERENTIAL_ODOMETRY_H
