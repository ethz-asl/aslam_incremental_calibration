/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file DiscreteDistribution.h
    \brief This file is an interface to the discrete distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_DISCRETEDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_DISCRETEDISTRIBUTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename X, int M = 1, int N = 1>
    class DiscreteDistribution;

  }
}

#include "aslam/calibration/statistics/DiscreteDistribution1v.h"
#include "aslam/calibration/statistics/DiscreteDistributionMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_DISCRETEDISTRIBUTION_H
