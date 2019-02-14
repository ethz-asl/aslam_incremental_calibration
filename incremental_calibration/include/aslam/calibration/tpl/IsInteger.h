/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file IsInteger.h
    \brief This file determines if types are integer
  */

#ifndef ASLAM_CALIBRATION_TPL_ISINTEGER_H
#define ASLAM_CALIBRATION_TPL_ISINTEGER_H

#include "aslam/calibration/tpl/Not.h"
#include "aslam/calibration/tpl/IsReal.h"

namespace aslam {
  namespace calibration {

    /** The IsInteger structure determines if a type is integer
        \brief Integer types definitions
      */
    template <typename T> struct IsInteger :
      public Not<typename IsReal<T>::Result> {
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_ISINTEGER_H
