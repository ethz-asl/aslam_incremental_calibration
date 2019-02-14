/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file DMIMeasurement.h
    \brief This file defines the DMIMeasurement structure which represents an
           Applanix DMI measurement.
  */

#ifndef ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H
#define ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H

namespace aslam {
  namespace calibration {

    /** The structure DMIMeasurement represents an Applanix DMI
        measurement.
        \brief Applanix DMI measurement.
      */
    struct DMIMeasurement {
      /** \name Public members
        @{
        */
      /// Wheel speed
      double wheelSpeed;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_DMI_MEASUREMENT_H
