/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file WheelSpeedsMeasurement.h
    \brief This file defines the WheelSpeedsMeasurement structure which
           represents a wheel speeds measurement.
  */

#ifndef ASLAM_CALIBRATION_CAR_WHEEL_SPEEDS_MEASUREMENT_H
#define ASLAM_CALIBRATION_CAR_WHEEL_SPEEDS_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure WheelSpeedsMeasurement represents a wheel speeds
        measurement.
        \brief Wheel speeds measurement.
      */
    struct WheelSpeedsMeasurement {
      /** \name Public members
        @{
        */
      /// Left speed measurement
      double left;
      /// Right speed measurement
      double right;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_WHEEL_SPEEDS_MEASUREMENT_H
