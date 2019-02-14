/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermObservation.h
    \brief This file defines the ErrorTermObservation class, which implements
           an observation model for the 2D-LRF problem.
  */

#ifndef ASLAM_CALIBRATION_2DLRF_ERROR_TERM_OBSERVATION_H
#define ASLAM_CALIBRATION_2DLRF_ERROR_TERM_OBSERVATION_H

#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
  namespace calibration {

    template <int M> class VectorDesignVariable;

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
      virtual void evaluateJacobiansImplementation(
        aslam::backend::JacobianContainer& J);
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
