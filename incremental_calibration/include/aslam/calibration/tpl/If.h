/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file If.h
    \brief This file defines the if template
  */

#ifndef ASLAM_CALIBRATION_TPL_IF_H
#define ASLAM_CALIBRATION_TPL_IF_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The If structure defines the If template
        \brief If template
      */
    template <typename C> struct If {
    };

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If template
      */
    template <> struct If<True> {
    public:
      /// Definition for true
      typedef True Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_IF_H
