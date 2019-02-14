/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousDistributionMv.h
    \brief This file contains an interface to the multivariate continuous
           distributions
  */

#include "aslam/calibration/functions/ContinuousFunction.h"
#include "aslam/calibration/statistics/Distribution.h"

namespace aslam {
  namespace calibration {

    /** The ContinuousDistributionMv class represents an interface to the
        multivariate continuous distributions.
        \brief Multivariate continuous distribution
      */
    template <typename X, int M, int N> class ContinuousDistribution :
      public ContinuousFunction<double, X, M, N>,
      public virtual Distribution<Eigen::Matrix<X, M, N> > {
    public:
      /** \name Types
        @{
        */
      /// Distribution type
      typedef ContinuousDistribution<X, M, N> DistributionType;
      /// Random variable type
      typedef Eigen::Matrix<X, M, N> RandomVariable;
      /// Mean type
      typedef Eigen::Matrix<double, M, N> Mean;
      /// Covariance type
      typedef Eigen::Matrix<double, M, M> Covariance;
      /// Mode type
      typedef Eigen::Matrix<X, M, N> Mode;
      /// Median type
      typedef Eigen::Matrix<double, M, N> Median;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~ContinuousDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the probablity density function at the given value
      virtual double pdf(const RandomVariable& value) const = 0;
      /// Interface to function
      virtual double getValue(const Eigen::Matrix<X, M, N>& argument) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/ContinuousDistributionMv.tpp"
