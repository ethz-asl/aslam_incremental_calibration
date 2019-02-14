/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousDistribution1v.h
    \brief This file contains an interface to the univariate continuous
           distributions
  */

#include "aslam/calibration/functions/ContinuousFunction.h"
#include "aslam/calibration/statistics/Distribution.h"

namespace aslam {
  namespace calibration {

    /** The ContinuousDistribution1v class represents an interface to the
        univariate continuous distributions.
        \brief Univariate continuous distribution
      */
    template <typename X> class ContinuousDistribution<X> :
      public ContinuousFunction<double, X>,
      public virtual Distribution<X> {
    public:
      /** \name Types
        @{
        */
      /// Distribution type
      typedef ContinuousDistribution<X> DistributionType;
      /// Random variable type
      typedef X RandomVariable;
      /// Mean type
      typedef double Mean;
      /// Variance type
      typedef double Variance;
      /// Mode type
      typedef X Mode;
      /// Median type
      typedef double Median;
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
      virtual double getValue(const X& argument) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/ContinuousDistribution1v.tpp"
