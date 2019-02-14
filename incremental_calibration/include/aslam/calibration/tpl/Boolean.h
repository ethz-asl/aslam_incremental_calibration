/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Boolean.h
    \brief This file defines the boolean template
  */

#ifndef ASLAM_CALIBRATION_TPL_BOOLEAN_H
#define ASLAM_CALIBRATION_TPL_BOOLEAN_H

namespace aslam {
  namespace calibration {

    /** The False structure defines the false template
        \brief False definition
      */
    struct False {
    public:
      /// False definition
      typedef False Result;
    };

    /** The True structure defines the true template
        \brief True definition
      */
    struct True {
    public:
      /// True definition
      typedef True Result;
      /// Numeric definition
      typedef bool Numeric;
    };

  }
}

#endif // ASLAM_CALIBRATION_TPL_BOOLEAN_H
