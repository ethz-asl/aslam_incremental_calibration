/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file CalibratorOptions.h
    \brief This file defines the CalibratorOptions structure.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_OPTIONS_H
#define ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_OPTIONS_H

#include <sm/timing/NsecTimeUtilities.hpp>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    /** The structure CalibratorOptions defines the options for the Calibrator
        class.
        \brief Calibrator options
      */
    struct CalibratorOptions {
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      CalibratorOptions();
      /// Constructs options from property tree
      CalibratorOptions(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Members
        @{
        */
      /// Window duration in seconds
      double windowDuration;
      /// Translation spline lambda
      double transSplineLambda;
      /// Rotation spline lambda
      double rotSplineLambda;
      /// Pose measurements per second desired for the spline
      int splineKnotsPerSecond;
      /// Translation spline order
      int transSplineOrder;
      /// Rotation spline order
      int rotSplineOrder;
      /// Variance for left wheel speed measurements
      double lwVariance;
      /// Variance for right wheel speed measurements
      double rwVariance;
      /// Variance for v_y constraint
      double vyVariance;
      /// Variance for v_z constraint
      double vzVariance;
      /// Verbose option
      bool verbose;
      /// Bound for time delay
      sm::timing::NsecTime delayBound;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_CALIBRATOR_OPTIONS_H
