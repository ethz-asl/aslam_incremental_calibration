/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousDistribution.h
    \brief This file is an interface to the continuous distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_CONTINUOUSDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_CONTINUOUSDISTRIBUTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename X, int M = 1, int N = 1>
    class ContinuousDistribution;

  }
}

#include "aslam/calibration/statistics/ContinuousDistribution1v.h"
#include "aslam/calibration/statistics/ContinuousDistributionMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_CONTINUOUSDISTRIBUTION_H
