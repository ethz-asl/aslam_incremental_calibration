/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/functions/LogGammaFunction.h"

#include "aslam/calibration/exceptions/BadArgumentException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LogGammaFunction<size_t>::LogGammaFunction() {
    }

    LogGammaFunction<size_t>::~LogGammaFunction() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double LogGammaFunction<size_t>::getValue(const VariableType& argument)
        const {
      if (argument)
        return LogFactorialFunction::getValue(argument - 1);
      else throw BadArgumentException<size_t>(argument,
        "LogGammaFunction<size_t>::getValue(): argument must be strictly "
        "positive",
        __FILE__, __LINE__);
    }

  }
}
