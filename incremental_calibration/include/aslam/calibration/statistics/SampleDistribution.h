/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file SampleDistribution.h
    \brief This file defines the class SampleDistribution, which represents an
           interface to sample distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H

#include <vector>

#include "aslam/calibration/statistics/Distribution.h"

namespace aslam {
  namespace calibration {

    /** The SampleDistribution class represents an interface to sample
        distributions, i.e. distributions which can directly be sampled from
        \brief Sample distribution
      */
    template <typename X> class SampleDistribution :
      public virtual Distribution<X> {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~SampleDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access a sample drawn from the distribution
      virtual X getSample() const = 0;
      /// Access samples drawn from the distribution
      void getSamples(std::vector<X>& samples, size_t numSamples) const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/SampleDistribution.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_SAMPLEDISTRIBUTION_H
