/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file GammaDistribution.h
    \brief This file defines the GammaDistribution class, which represents
           a gamma distribution
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_GAMMADISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_GAMMADISTRIBUTION_H

#include "aslam/calibration/statistics/ContinuousDistribution.h"
#include "aslam/calibration/statistics/SampleDistribution.h"
#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The GammaDistribution class represents a gamma distribution,
        i.e., a continuous distribution that models waiting times, e.g., the
        waiting time until death. It is also a conjugate prior to the
        Poisson/Exponential/... distributions.
        \brief Gamma distribution
      */
    template <typename T = double> class GammaDistribution :
      public ContinuousDistribution<double>,
      public SampleDistribution<double>,
      public virtual Serializable {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs distribution from parameters
      GammaDistribution(const T& shape = T(1), double invScale = 1.0);
      /// Copy constructor
      GammaDistribution(const GammaDistribution& other);
      /// Assignment operator
      GammaDistribution& operator = (const GammaDistribution& other);
      /// Destructor
      virtual ~GammaDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the shape parameter
      void setShape(const T& shape);
      /// Returns the shape parameter
      const T& getShape() const;
      /// Sets the inverse scale parameter
      void setInvScale(double invScale);
      /// Returns the inverse scale parameter
      double getInvScale() const;
      /// Returns the normalizer
      double getNormalizer() const;
      /// Returns the mean of the distribution
      Mean getMean() const;
      /// Returns the mode of the distribution
      Mode getMode() const;
      /// Returns the variance of the distribution
      Variance getVariance() const;
      /// Access the probablity density function at the given value
      virtual double pdf(const RandomVariable& value) const;
      /// Access the log-probablity density function at the given value
      double logpdf(const RandomVariable& value) const;
      /// Access the cumulative density function at the given value
      double cdf(const RandomVariable& value) const;
      /// Access the inverse cumulative density function at the given value
      RandomVariable invcdf(double probability) const;
      /// Access a sample drawn from the distribution
      virtual RandomVariable getSample() const;
      /// Returns the KL-divergence with another distribution
      double KLDivergence(const GammaDistribution<T>& other) const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Compute normalizer
      void computeNormalizer();
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Shape parameter
      T mShape;
      /// Inverse scale parameter
      double mInvScale;
      /// Normalizer
      double mNormalizer;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/GammaDistribution.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_GAMMADISTRIBUTION_H
