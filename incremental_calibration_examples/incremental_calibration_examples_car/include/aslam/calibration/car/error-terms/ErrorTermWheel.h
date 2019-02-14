/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermWheel.h
    \brief This file defines the ErrorTermWheel class, which implements an error
           term for a wheel.
  */

#ifndef ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H
#define ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H

#include <Eigen/Core>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/ScalarExpression.hpp>

namespace aslam {
  namespace calibration {

    /** The class ErrorTermWheel implements an error term for a wheel.
        \brief Wheel error term
      */
    class ErrorTermWheel :
      public aslam::backend::ErrorTermFs<3> {
    public:
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix3d Covariance;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /**
       * Constructs the error term from input data and design variable.
       * \brief Constructs the error term
       *
       * @param v_v_mw \f$^{v}\mathbf{v}^{mw}\f$ linear velocity of
       *   the wheel w.r. to mapping frame, expressed in vehicle frame
       * @param k scaling factor
       * @param measurement odometry measurement (\f$v\f$)
       * @param sigma2Wheel covariance matrix of the odometry measurement
       * @param frontEnabled front wheel flag
       */
      ErrorTermWheel(const aslam::backend::EuclideanExpression& v_v_mw, const
        aslam::backend::ScalarExpression& k, double measurement, const
        Covariance& sigma2Wheel, bool frontEnabled = false);
      /// Copy constructor
      ErrorTermWheel(const ErrorTermWheel& other);
      /// Assignment operator
      ErrorTermWheel& operator = (const ErrorTermWheel& other);
      /// Destructor
      virtual ~ErrorTermWheel();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the measurement
      double getMeasurement() const;
      /// Sets the measurement
      void setMeasurement(double measurement);
      /// Returns the covariance of the measurement
      const Covariance& getCovariance() const;
      /// Sets the covariance of the measurement
      void setCovariance(const Covariance& sigma2Wheel);
      /// Returns the front wheel flag
      bool getFrontEnabled() const;
      /// Sets the front wheel flag
      void setFrontEnabled(bool enabled = true);
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
       * Design variable representing the linear velocity of the wheel
       * w.r. to the mapping frame, expressed in the vehicle frame.
       * \brief Scaled linear velocity
       */
      aslam::backend::EuclideanExpression _v_v_mw;
      /// Scaling factor
      aslam::backend::ScalarExpression _k;
      /// Measured odometry
      double _measurement;
      /// Covariance of the measurement
      Covariance _sigma2Wheel;
      /// Front wheel flag
      bool _frontEnabled;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_ERROR_TERM_WHEEL_H
