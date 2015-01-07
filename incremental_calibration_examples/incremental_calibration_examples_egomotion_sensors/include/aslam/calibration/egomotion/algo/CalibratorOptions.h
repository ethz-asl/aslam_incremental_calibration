/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file CalibratorOptions.h
    \brief This file defines the CalibratorOptions structure.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_OPTIONS_H
#define ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_OPTIONS_H

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
      /// Verbose option
      bool verbose;
      /// Bound for time delay in nanoseconds
      sm::timing::NsecTime delayBound;
      /// Reference sensor
      size_t referenceSensor;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_CALIBRATOR_OPTIONS_H
