/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file DiscreteDistributionMv.h
    \brief This file contains an interface to the multivariate discrete
           distributions
  */

#include "aslam/calibration/functions/DiscreteFunction.h"
#include "aslam/calibration/statistics/Distribution.h"

namespace aslam {
  namespace calibration {

    /** The DiscreteDistributionMv class represents an interface to the
        multivariate discrete distributions.
        \brief Multivariate discrete distribution
      */
    template <typename X, int M, int N> class DiscreteDistribution :
      public DiscreteFunction<double, X, M, N>,
      public virtual Distribution<Eigen::Matrix<X, M, N> > {
    public:
      /** \name Types
        @{
        */
      /// Distribution type
      typedef DiscreteDistribution<X, M, N> DistributionType;
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
      virtual ~DiscreteDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the probablity of the random variable to take on the given val.
      virtual double pmf(const RandomVariable& value) const = 0;
      /// Interface to function
      virtual double getValue(const Eigen::Matrix<X, M, N>& argument) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/DiscreteDistributionMv.tpp"
