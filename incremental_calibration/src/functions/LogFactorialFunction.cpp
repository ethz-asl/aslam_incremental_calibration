/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/functions/LogFactorialFunction.h"

#include <cmath>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LogFactorialFunction::LogFactorialFunction() {
    }

    LogFactorialFunction::~LogFactorialFunction() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double LogFactorialFunction::getValue(const VariableType& argument) const {
      if (argument) {
        double value = 0.0;
        for (size_t x = 1; x < argument; ++x)
          value += log(x + 1);
        return value;
      }
      else
        return 0.0;
    }

  }
}
