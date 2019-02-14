/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file IsReal.h
    \brief This file determines if types are real-valued
  */

#ifndef ASLAM_CALIBRATION_TPL_ISREAL_H
#define ASLAM_CALIBRATION_TPL_ISREAL_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <typename T> struct IsReal {
    public:
      /// Non-real types definition
      typedef False Result;
    };

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <> struct IsReal<float> {
    public:
      /// Real type definition
      typedef True Result;
    };

    /** The IsReal structure determines if a type is real
        \brief Real types definitions
      */
    template <> struct IsReal<double> {
    public:
      /// Real type definition
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_ISREAL_H
