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

/** \file ErrorTermPose.h
    \brief This file defines the ErrorTermPose class, which implements
           an error term for a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/TransformationExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermPose implements an error term for a pose sensor
        such as an Applanix.
        \brief Pose error term
      */
    class ErrorTermPose :
      public aslam::backend::ErrorTermFs<6> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 6, 6> Covariance;
      /// Measurement type
      typedef Eigen::Matrix<double, 6, 1> Input;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param T pose in world frame
       * @param xm pose measurement (\f$[x,y,z,\theta_z,\theta_y,\theta_x]\f$)
       * @param Q Covariance matrix of the pose measurement
       */
      ErrorTermPose(const aslam::backend::TransformationExpression& T,
        const Input& xm, const Covariance& Q);
      /// Copy constructor
      ErrorTermPose(const ErrorTermPose& other);
      /// Assignment operator
      ErrorTermPose& operator = (const ErrorTermPose& other);
      /// Destructor
      virtual ~ErrorTermPose();
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
      void setInput(const Input& xm);
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
        aslam::backend::JacobianContainer & _jacobians);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Estimated pose
      aslam::backend::TransformationExpression _T;
      /// Measured pose
      Input _xm;
      /// Covariance matrix of the pose measurement
      Covariance _Q;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_POSE_H
