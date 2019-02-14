/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file CarCalibratorOptions.h
    \brief This file defines the CarCalibratorOptions structure.
  */

#ifndef ASLAM_CALIBRATION_CAR_CALIBRATOR_OPTIONS_H
#define ASLAM_CALIBRATION_CAR_CALIBRATOR_OPTIONS_H

#include <cstdint>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace calibration {

    /** The structure CarCalibratorOptions defines the options for the
        CarCalibrator class.
        \brief Car calibrator options
      */
    struct CarCalibratorOptions {
      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      CarCalibratorOptions();
      /// Constructs options from property tree
      CarCalibratorOptions(const sm::PropertyTree& config);
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
      /// Tolerance for rejecting low speed measurements
      double linearVelocityTolerance;
      /// Percent error for DMI measurements
      double dmiPercentError;
      /// Variance for DMI measurements
      double dmiVariance;
      /// Percent error for front left wheel speed measurements
      double flwPercentError;
      /// Variance for front left wheel speed measurements
      double flwVariance;
      /// Percent error for front right wheel speed measurements
      double frwPercentError;
      /// Variance for front right wheel speed measurements
      double frwVariance;
      /// Percent error for rear left wheel speed measurements
      double rlwPercentError;
      /// Variance for rear left wheel speed measurements
      double rlwVariance;
      /// Percent error for rear right wheel speed measurements
      double rrwPercentError;
      /// Variance for rear right wheel speed measurements
      double rrwVariance;
      /// Variance for steering measurement
      double steeringVariance;
      /// Wheel speed sensor cutoff
      uint16_t wheelSpeedSensorCutoff;
      /// Variance for v_y constraint
      double vyVariance;
      /// Variance for v_z constraint
      double vzVariance;
      /// Verbose option
      bool verbose;
      /// Use pose error terms
      bool usePose;
      /// Use velocities error terms
      bool useVelocities;
      /// Bound for time delay
      sm::timing::NsecTime delayBound;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_CALIBRATOR_OPTIONS_H
