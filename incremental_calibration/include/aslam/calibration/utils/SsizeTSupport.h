/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file SsizeTSupport.h
    \brief This file defines the Eigen support for ssize_t type.
  */

#ifndef ASLAM_CALIBRATION_UTILS_SSIZETSUPPORT_H
#define ASLAM_CALIBRATION_UTILS_SSIZETSUPPORT_H

#include <cstdlib>

#include <Eigen/Core>

namespace Eigen {

  /** The NumTraits<ssize_t> structure defines support for ssize_t type in
      Eigen.
      \brief Eigen support for ssize_t
    */
  template<> struct NumTraits<ssize_t> {
    /// Real definition
    typedef ssize_t Real;
    /// Floating point definition
    typedef double FloatingPoint;
    /// Enum for Eigen
    enum {
      /// Is complex
      IsComplex = 0,
      /// Has floating point
      HasFloatingPoint = 0,
      /// Read cost
      ReadCost = 1,
      /// Add cost
      AddCost = 1,
      /// Multiplicative cost
      MulCost = 1,
    };
  };

}

#endif // ASLAM_CALIBRATION_UTILS_SSIZETSUPPORT_H
