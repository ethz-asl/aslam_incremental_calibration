/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file EstimatorML.h
    \brief This file defines the EstimatorML class, which implements
           maximum likelihood estimators for various distributions.
  */

#ifndef ASLAM_CALIBRATION_STATISTICS_ESTIMATORML_H
#define ASLAM_CALIBRATION_STATISTICS_ESTIMATORML_H

namespace aslam {
  namespace calibration {

    template <typename D> class EstimatorML;

  }
}

#include "aslam/calibration/statistics/EstimatorMLNormal1v.h"
#include "aslam/calibration/statistics/EstimatorMLNormalMv.h"

#endif // ASLAM_CALIBRATION_STATISTICS_ESIMATORML_H
