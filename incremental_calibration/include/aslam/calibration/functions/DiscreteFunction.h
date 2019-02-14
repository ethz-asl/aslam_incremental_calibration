/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file DiscreteFunction.h
    \brief This file is an interface to the discrete functions
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H

#include <cstdlib>

namespace aslam {
  namespace calibration {

    template <typename Y, typename X, int M = 1, int N = 1>
      class DiscreteFunction;

  }
}

#include "aslam/calibration/functions/DiscreteFunction1v.h"
#include "aslam/calibration/functions/DiscreteFunctionMv.h"

#endif // ASLAM_CALIBRATION_FUNCTIONS_DISCRETEFUNCTION_H
