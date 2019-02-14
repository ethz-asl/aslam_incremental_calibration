/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file IfThenElse.h
    \brief This file defines the If-Then-Else template
  */

#ifndef ASLAM_CALIBRATION_TPL_IFTHENELSE_H
#define ASLAM_CALIBRATION_TPL_IFTHENELSE_H

#include "aslam/calibration/tpl/Boolean.h"

namespace aslam {
  namespace calibration {

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename C, typename A, typename B> struct IfThenElse;

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename A, typename B> struct IfThenElse<True, A, B> {
    public:
      /// Definition for If selection
      typedef A Result;
    };

    /** The IfThenElse structure defines the If-Then-Else template
        \brief If-Then-Else template
      */
    template <typename A, typename B> class IfThenElse<False, A, B> {
    public:
      /// Definition for Else selection
      typedef B Result;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_IFTHENELSE_H
