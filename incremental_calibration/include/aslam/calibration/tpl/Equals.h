/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Equals.h
    \brief This file defines the equality testing template
  */

#ifndef ASLAM_CALIBRATION_TPL_EQUALS_H
#define ASLAM_CALIBRATION_TPL_EQUALS_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The Equals structure defines the equality testing template
        \brief Equality testing template
      */
    template <typename A, typename B> struct Equals {
    public:
      /// Defines the not equal
      typedef False Result;
    };

    /** The Equals structure defines the equality testing template
        \brief Equality testing template
      */
    template <typename A> struct Equals<A, A> {
    public:
      /// Defines the equal
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_EQUALS_H
