/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Distribution.h
    \brief This file contains an interface to any kind of distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The Distribution class represents an interface to any kind of
        distributions.
        \brief Distribution
      */
    template <typename X> class Distribution :
      public virtual Function<double, X> {
    public:
      /** \name Types
        @{
        */
      /// Random variable type
      typedef typename Function<double, X>::Domain RandomVariable;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~Distribution();
      /** @}
        */

    };

  }
}

#include "aslam/calibration/statistics/Distribution.tpp"

#endif // ASLAM_CALIBRATION_STATISTICS_DISTRIBUTION_H
