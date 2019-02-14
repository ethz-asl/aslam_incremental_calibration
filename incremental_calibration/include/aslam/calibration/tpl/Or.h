/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Or.h
    \brief This file defines the or template
  */

#ifndef ASLAM_CALIBRATION_TPL_OR_H
#define ASLAM_CALIBRATION_TPL_OR_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The Or structure defines the Or template
        \brief Or template
      */
    template <typename A, typename B> struct Or {
    public:
      /// Defines true
      typedef True Result;
    };

    /** The Or structure defines the Or template
        \brief Or template
      */
    template <> struct Or<False, False> {
    public:
      /// Defines false
      typedef False Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_OR_H
