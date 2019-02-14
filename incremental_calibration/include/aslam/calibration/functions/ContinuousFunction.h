/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file ContinuousFunction.h
    \brief This file is an interface to the continuous functions
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_CONTINUOUSFUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_CONTINUOUSFUNCTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename Y, typename X, int M = 1, int N = 1>
      class ContinuousFunction;

  }
}

#include "aslam/calibration/functions/ContinuousFunction1v.h"
#include "aslam/calibration/functions/ContinuousFunctionMv.h"

#endif // ASLAM_CALIBRATION_FUNCTIONS_CONTINUOUSFUNCTION_H
