/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/egomotion/algo/CalibratorOptions.h"

#include <sm/PropertyTree.hpp>

using namespace sm;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CalibratorOptions::CalibratorOptions() :
        windowDuration(10.0),
        transSplineLambda(0.0),
        rotSplineLambda(0.0),
        splineKnotsPerSecond(5),
        transSplineOrder(4),
        rotSplineOrder(4),
        verbose(true),
        delayBound(50000000),
        referenceSensor(0) {
    }

    CalibratorOptions::CalibratorOptions(const PropertyTree& config) {
      windowDuration = config.getDouble("windowDuration");
      verbose = config.getBool("verbose");
      delayBound = config.getInt("delayBound");
      referenceSensor = config.getInt("referenceSensor");

      transSplineLambda = config.getDouble("splines/transSplineLambda");
      rotSplineLambda = config.getDouble("splines/rotSplineLambda");
      splineKnotsPerSecond = config.getInt("splines/splineKnotsPerSecond");
      transSplineOrder = config.getInt("splines/transSplineOrder",
        transSplineOrder);
      rotSplineOrder = config.getInt("splines/rotSplineOrder");
    }

  }
}
