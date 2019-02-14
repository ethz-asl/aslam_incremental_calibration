/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file And.h
    \brief This file defines the and template
  */

#ifndef ASLAM_CALIBRATION_TPL_AND_H
#define ASLAM_CALIBRATION_TPL_AND_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The And structure defines the and template
        \brief And template
      */
    template <typename A, typename B> struct And {
    public:
      /// Defines and resulting to false
      typedef False Result;
    };

    /** The And structure defines the and template
        \brief And template
      */
    template <> struct And<True, True> {
    public:
      /// Defines and resulting to true
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_AND_H
