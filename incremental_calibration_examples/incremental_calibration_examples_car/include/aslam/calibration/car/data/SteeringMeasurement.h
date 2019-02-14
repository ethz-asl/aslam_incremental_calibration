/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file SteeringMeasurement.h
    \brief This file defines the SteeringMeasurement structure which
           represents a steering measurement.
  */

#ifndef ASLAM_CALIBRATION_CAR_STEERING_MEASUREMENT_H
#define ASLAM_CALIBRATION_CAR_STEERING_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure SteeringMeasurement represents a steering measurement.
        \brief Steering measurement.
      */
    struct SteeringMeasurement {
      /** \name Members
        @{
        */
      /// Measurement
      double value;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_STEERING_MEASUREMENT_H
