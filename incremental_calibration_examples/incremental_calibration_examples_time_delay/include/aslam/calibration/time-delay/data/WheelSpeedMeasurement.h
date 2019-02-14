/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file WheelSpeedsMeasurement.h
    \brief This file defines the WheelSpeedsMeasurement structure which
           represents a wheel speed measurement.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_WHEEL_SPEED_MEASUREMENT_H
#define ASLAM_CALIBRATION_TIME_DELAY_WHEEL_SPEED_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure WheelSpeedsMeasurement represents a wheel speeds
        measurement.
        \brief Wheel speed measurement.
      */
    struct WheelSpeedMeasurement {
      /** \name Public members
        @{
        */
      /// Measurement
      double value;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_WHEEL_SPEED_MEASUREMENT_H
