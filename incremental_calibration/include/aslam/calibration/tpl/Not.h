/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Not.h
    \brief This file defines the not template
  */

#ifndef ASLAM_CALIBRATION_TPL_NOT_H
#define ASLAM_CALIBRATION_TPL_NOT_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The Not structure defines the Not template
        \brief Not template
      */
    template <typename A> struct Not;

    /** The Not structure defines the Not template
        \brief Not template
      */
    template <> struct Not<False> {
    public:
      /// Defines not false
      typedef True Result;
    };

    /** The Not structure defines the Not template
        \brief Not template
      */
    template <> struct Not<True> {
    public:
      /// Defines not true
      typedef False Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_NOT_H
