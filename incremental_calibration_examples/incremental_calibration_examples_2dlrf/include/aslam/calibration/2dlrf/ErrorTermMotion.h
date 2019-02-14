/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ErrorTermMotion.h
    \brief This file defines the ErrorTermMotion class, which implements
           a motion model for the 2D-LRF problem.
  */

#ifndef ASLAM_CALIBRATION_2DLRF_ERROR_TERM_MOTION_H
#define ASLAM_CALIBRATION_2DLRF_ERROR_TERM_MOTION_H

#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
  namespace calibration {

    template <int M> class VectorDesignVariable;

    /** The class ErrorTermMotion implements a motion model for the 2D-LRF
        problem.
        \brief 2D-LRF motion model
      */
    class ErrorTermMotion :
      public aslam::backend::ErrorTermFs<3> {
    public:
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \name Types definitions
        @{
        */
      /// Covariance type
      typedef Eigen::Matrix<double, 3, 3> Covariance;
      /// Input type
      typedef Eigen::Matrix<double, 3, 1> Input;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      ErrorTermMotion(VectorDesignVariable<3>* xkm1,
        VectorDesignVariable<3>* xk, double T, const Input& uk,
        const Covariance& Q);
      /// Copy constructor
      ErrorTermMotion(const ErrorTermMotion& other);
      /// Assignment operator
      ErrorTermMotion& operator = (const ErrorTermMotion& other);
      /// Destructor
      virtual ~ErrorTermMotion();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the timestep
      double getTimestep() const;
      /// Sets the timestep
      void setTimestep(double T);
      /// Returns the input
      const Input& getInput() const;
      /// Returns the input
      Input& getInput();
      /// Sets the input
      void setInput(const Input& uk);
      /// Returns the covariance
      const Covariance& getCovariance() const;
      /// Returns the covariance
      Covariance& getCovariance();
      /// Sets the covariance
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
        aslam::backend::JacobianContainer& J);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// State at time k-1
      VectorDesignVariable<3>* _xkm1;
      /// State at time k
      VectorDesignVariable<3>* _xk;
      /// Timestep size
      double _T;
      /// Input at time k
      Input _uk;
      /// Covariance matrix
      Covariance _Q;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_2DLRF_ERROR_TERM_MOTION_H
