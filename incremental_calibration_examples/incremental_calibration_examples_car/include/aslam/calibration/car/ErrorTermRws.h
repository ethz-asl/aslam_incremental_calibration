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

/** \file ErrorTermRws.h
    \brief This file defines the ErrorTermRws class, which implements
           an error term for the rear wheel speeds.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_RWS_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_RWS_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace aslam {
  namespace calibration {

    template <int M> class VectorDesignVariable;

    /** The class ErrorTermRws implements an error term for rear wheel speeds.
        \brief Rear wheel speeds error term
      */
    class ErrorTermRws :
      public aslam::backend::ErrorTermFs<2> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix2d Covariance;
      /// Measurement type
      typedef Eigen::Vector2d Input;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param v_oo \f$\mathbf{v}_{oo}\f$ linear velocity in odometry frame
       * @param om_oo \f$\boldsymbol{\omega}_{oo}\f$ angular velocity in
       *              odometry frame
       * @param params odometry parameters (\f$[L,e_r,e_f,a_0,a_1,a_2,a_3,
       *        \kappa_{rl},\kappa_{rr},\kappa_{fl},\kappa_{fr}]\f$)
       * @param odo odometry measurement
       *        (\f$[v_{rl},v_{rr}]\f$)
       * @param Q Covariance matrix of the odometry measurement
       */
      ErrorTermRws(const aslam::backend::EuclideanExpression& v_oo,
        const aslam::backend::EuclideanExpression& om_oo,
        VectorDesignVariable<11>* params,
        const Input& odo, const Covariance& Q);
      /// Copy constructor
      ErrorTermRws(const ErrorTermRws& other);
      /// Assignment operator
      ErrorTermRws& operator = (const ErrorTermRws& other);
      /// Destructor
      virtual ~ErrorTermRws();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the input measurement
      const Input& getInput() const;
      /// Returns the input measurement
      Input& getInput();
      /// Sets the input measurement
      void setInput(const Input& odo);
      /// Returns the covariance of the measurement
      const Covariance& getCovariance() const;
      /// Returns the covariance of the measurement
      Covariance& getCovariance();
      /// Sets the covariance of the measurement
      void setCovariance(const Covariance& Q);
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
      /// Estimated vehicle linear velocity in odometry frame
      aslam::backend::EuclideanExpression _v_oo;
      /// Estimated vehicle angular velocity in odometry frame
      aslam::backend::EuclideanExpression _om_oo;
      /// Estimated odometry parameters
      VectorDesignVariable<11>* _params;
      /// Measured odometry
      Input _odo;
      /// Covariance matrix of the odometry measurement
      Covariance _Q;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_RWS_H
