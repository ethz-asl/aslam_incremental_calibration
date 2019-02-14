/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file MeasurementsContainer.h
    \brief This file defines a generic container for measurements.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_MEASUREMENTS_CONTAINER_H
#define ASLAM_CALIBRATION_EGOMOTION_MEASUREMENTS_CONTAINER_H

#include <vector>
#include <utility>

#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace calibration {

    /** The structure MeasurementsContainer represents a generic measurements
        container.
        \brief Measurements container
      */
    template <typename C> struct MeasurementsContainer {
      /** \name Types definitions
        @{
        */
      /// Container type
      typedef std::vector<std::pair<sm::timing::NsecTime, C> > Type;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_MEASUREMENTS_CONTAINER_H
