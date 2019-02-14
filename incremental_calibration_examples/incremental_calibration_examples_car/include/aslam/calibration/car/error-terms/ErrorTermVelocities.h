/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermVelocities.h
    \brief This file defines the ErrorTermVelocities class, which implements an
           error term for velocities returned by a pose sensor.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_VELOCITIES_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_VELOCITIES_H

#include <Eigen/Core>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermVelocities implements an error term for velocities
        returned by a pose sensor such as the Applanix.
        \brief Velocities error term
      */
    class ErrorTermVelocities :
      public aslam::backend::ErrorTermFs<6> {
    public:
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 3, 3> Covariance;
      /// Measurement type
      typedef Eigen::Matrix<double, 3, 1> Input;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /** 
       * Constructs the error term from input data and design variables
       * \brief Constructs the error term
       * 
       * @param r_v_mr linear velocity of the frame R w.r. to M, in R
       * @param r_om_mr angular velocity of the frame R w.r. to M, in R
       * @param r_v_mr_m measured linear velocity
       * @param r_om_mr_m measured angular velocity
       * @param sigma2_v covariance matrix of r_v_mr_m
       * @param sigma2_om covariance matrix of r_om_mr_m
       */
      ErrorTermVelocities(const aslam::backend::EuclideanExpression& r_v_mr,
        const aslam::backend::EuclideanExpression& r_om_mr, const Input&
        r_v_mr_m, const Input& r_om_mr_m, const Covariance& sigma2_v, const
        Covariance& sigma2_om);
      /// Copy constructor
      ErrorTermVelocities(const ErrorTermVelocities& other);
      /// Assignment operator
      ErrorTermVelocities& operator = (const ErrorTermVelocities& other);
      /// Destructor
      virtual ~ErrorTermVelocities();
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
      /// Linear velocity
      aslam::backend::EuclideanExpression _r_v_mr;
      /// Angular velocity
      aslam::backend::EuclideanExpression _r_om_mr;
      /// Measured linear velocity
      Input _r_v_mr_m;
      /// Measured angular velocity
      Input _r_om_mr_m;
      /// Covariance matrix of r_v_mr_m
      Covariance _sigma2_v;
      /// Covariance matrix of r_om_mr_m
      Covariance _sigma2_om;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_VELOCITIES_H
