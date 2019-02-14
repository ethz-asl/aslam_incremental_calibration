/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Histogram.h
    \brief This file is an interface to histograms.
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_HISTOGRAM_H
#define ASLAM_CALIBRATION_STATISTICS_HISTOGRAM_H

namespace aslam {
  namespace calibration {

    template <typename T, int M = 1> class Histogram;

  }
}

#include "aslam/calibration/statistics/Histogram1v.h"
#include "aslam/calibration/statistics/HistogramMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_HISTOGRAM_H
