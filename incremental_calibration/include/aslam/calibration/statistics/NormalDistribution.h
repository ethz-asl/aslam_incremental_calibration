/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file NormalDistribution.h
    \brief This file is an interface to the normal distributions
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H
#define ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H

namespace aslam {
  namespace calibration {

    template <int M = 1> class NormalDistribution;

  }
}

#include "aslam/calibration/statistics/NormalDistribution1v.h"
#include "aslam/calibration/statistics/NormalDistributionMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_NORMALDISTRIBUTION_H
