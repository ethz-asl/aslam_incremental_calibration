/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ChiSquareDistribution.h
    \brief This file defines the ChiSquareDistribution class, which represents
           a chi-square distribution
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_CHISQUAREDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_CHISQUAREDISTRIBUTION_H

#include "aslam/calibration/statistics/GammaDistribution.h"

namespace aslam {
  namespace calibration {

    /** The ChiSquareDistribution class represents a chi-square distribution,
        i.e., a continuous distribution that models the distribution of a sum
        of the squares of k independent standard normal random variables
        (k degrees).
        \brief Chi-Square distribution
      */
    class ChiSquareDistribution :
      public GammaDistribution<> {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs distribution from parameters
      ChiSquareDistribution(double degrees = 1);
      /// Copy constructor
      ChiSquareDistribution(const ChiSquareDistribution& other);
      /// Assignment operator
      ChiSquareDistribution& operator = (const ChiSquareDistribution& other);
      /// Destructor
      virtual ~ChiSquareDistribution();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Sets the degrees of freedom of the distribution
      void setDegrees(double degrees);
      /// Returns the degrees of freedom of the distribution
      double getDegrees() const;
      /// Returns the median of the distribution
      Median getMedian() const;
      /** @}
        */

    protected:
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

    };

  }
}

#endif // ASLAM_CALIBRATION_STATISTICS_CHISQUAREDISTRIBUTION_H
