/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file UniformDistribution.h
    \brief This file is an interface to the uniform distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_UNIFORMDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_UNIFORMDISTRIBUTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename X, int M = 1> class UniformDistribution;

  }
}

#include "aslam/calibration/statistics/UniformDistribution1v.h"
#include "aslam/calibration/statistics/UniformDistributionMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_UNIFORMDISTRIBUTION_H
