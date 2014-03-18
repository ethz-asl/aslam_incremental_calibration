/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

/** \file ErrorTermSteering.h
    \brief This file defines the ErrorTermSteering class, which implements an
           error term for the steering angle.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_STEERING_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_STEERING_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace aslam {
  namespace calibration {

    template <int M> class VectorDesignVariable;

    /** The class ErrorTermSteering implements an error term for the steering.
        \brief Steering error term
      */
    class ErrorTermSteering :
      public aslam::backend::ErrorTermFs<1> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Constructors/destructor
        @{
        */
      /**
       * Constructs the error term from input data and design variable.
       * \brief Constructs the error term
       *
       * @param v_v_mw_r \f$^{v}\mathbf{v}^{mw}\f$ linear velocity of
       *   virtual wheel w.r. to mapping frame, expressed in vehicle frame
       * @param measurement odometry measurement (\f$\varphi\f$)
       * @param sigma2 variance matrix of the odometry measurement
       * @param params parameters for the steering conversion
       */
      ErrorTermSteering(const aslam::backend::EuclideanExpression& v_v_mw,
        double measurement, double sigma2, VectorDesignVariable<4>* params);
      /// Copy constructor
      ErrorTermSteering(const ErrorTermSteering& other);
      /// Assignment operator
      ErrorTermSteering& operator = (const ErrorTermSteering& other);
      /// Destructor
      virtual ~ErrorTermSteering();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the measurement
      double getMeasurement() const;
      /// Sets the measurement
      void setMeasurement(double measurement);
      /// Returns the variance of the measurement
      double getVariance() const;
      /// Sets the variance of the measurement
      void setVariance(double sigma2);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Evaluate the error term and return the weighted squared error
      virtual double evaluateErrorImplementation();
      /// Evaluate the Jacobians
      virtual void evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& _jacobians);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /**
       * Design variable representing the linear velocity of the virtual wheel
       *   w.r. to the mapping frame, expressed in the wheel frame.
       * \brief Linear velocity
       */
      aslam::backend::EuclideanExpression _v_v_mw;
      /// Measured odometry
      double _measurement;
      /// variance of the measurement
      double _sigma2;
      /// Steering wheel conversion coefficients
      VectorDesignVariable<4>* _params;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_STEERING_H
