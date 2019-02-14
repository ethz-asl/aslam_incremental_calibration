/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file IsVoid.h
    \brief This file determines if a type is void
  */

#ifndef ASLAM_CALIBRATION_TPL_ISVOID_H
#define ASLAM_CALIBRATION_TPL_ISVOID_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The IsVoid structure determines if a type is void
        \brief Void types definitions
      */
    template <typename T> struct IsVoid {
    public:
      /// Result for non-void types
      typedef False Result;
    };

    /** The IsVoid structure determines if a type is void
        \brief Void types definitions
      */
    template <> struct IsVoid<void> {
    public:
      /// Result for void types
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_ISVOID_H
