/******************************************************************************
 * Copyright (C) 2012 by Jerome Maye                                          *
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

/** \file ErrorTermObservation.h
    \brief This file defines the ErrorTermObservation class, which implements
           an observation model for the 2D-LRF problem.
  */

#ifndef ASLAM_CALIBRATION_2DLRF_ERROR_TERM_OBSERVATION_H
#define ASLAM_CALIBRATION_2DLRF_ERROR_TERM_OBSERVATION_H

#include <vector>

#include <aslam/backend/ErrorTerm.hpp>

#include "aslam/calibration/data-structures/VectorDesignVariable.h"

namespace aslam {
  namespace calibration {

    /** The class ErrorTermObservation implements an observation model for the
        2D-LRF problem.
        \brief 2D-LRF observation model
      */
    class ErrorTermObservation :
      public aslam::backend::ErrorTermFs<2> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 2, 2> Covariance;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      ErrorTermObservation(VectorDesignVariable<3>* xk,
        VectorDesignVariable<2>* xl, VectorDesignVariable<3>* Theta, double r,
        double b, const Covariance& R);
      /// Copy constructor
      ErrorTermObservation(const ErrorTermObservation& other);
      /// Assignment operator
      ErrorTermObservation& operator = (const ErrorTermObservation& other);
      /// Destructor
      virtual ~ErrorTermObservation();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the range measurement
      double getRange() const;
      /// Sets the range measurement
      void setRange(double r);
      /// Returns the bearing measurement
      double getBearing() const;
      /// Sets the bearing measurement
      void setBearing(double b);
      /// Returns the covariance
      const Covariance& getCovariance() const;
      /// Returns the covariance
      Covariance& getCovariance();
      /// Sets the covariance
      void setCovariance(const Covariance& R);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Evaluate the error term and return the weighted squared error
      virtual double evaluateErrorImplementation();
      /// Evaluate the Jacobians
      virtual void evaluateJacobiansImplementation();
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// State at time k
      VectorDesignVariable<3>* _xk;
      /// Landmark position
      VectorDesignVariable<2>* _xl;
      /// Calibration parameters
      VectorDesignVariable<3>* _Theta;
      /// Range measurement
      double _r;
      /// Bearing measurement
      double _b;
      /// Covariance matrix
      Covariance _R;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_2DLRF_ERROR_TERM_OBSERVATION_H
