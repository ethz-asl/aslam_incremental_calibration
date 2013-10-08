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

/** \file ErrorTermVehicleModel.h
    \brief This file defines the ErrorTermVehicleModel class, which implements
           an error term for the vehicle model.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_VEHICLE_MODEL_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_VEHICLE_MODEL_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermVehicleModel implements an error term for the vehicle
        model.
        \brief Vehicle model error term
      */
    class ErrorTermVehicleModel :
      public aslam::backend::ErrorTermFs<4> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix4d Covariance;
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
       * @param Q Covariance matrix of the pseudo-measurement
       */
      ErrorTermVehicleModel(const aslam::backend::EuclideanExpression& v_oo,
        const aslam::backend::EuclideanExpression& om_oo, const Covariance& Q);
      /// Copy constructor
      ErrorTermVehicleModel(const ErrorTermVehicleModel& other);
      /// Assignment operator
      ErrorTermVehicleModel& operator = (const ErrorTermVehicleModel& other);
      /// Destructor
      virtual ~ErrorTermVehicleModel();
      /** @}
        */

      /** \name Accessors
        @{
        */
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
      virtual void evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& _jacobians);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Estimated vehicle linear velocity in odometry frame
      aslam::backend::EuclideanExpression _v_oo;
      /// Estimated vehicle angular velocity in odometry frame
      aslam::backend::EuclideanExpression _om_oo;
      /// Covariance matrix of the pseudo-measurement
      Covariance _Q;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_VEHICLE_MODEL_H
