/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/car/algo/CarCalibratorOptions.h"

#include <sm/PropertyTree.hpp>

using namespace sm;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    CarCalibratorOptions::CarCalibratorOptions() :
        windowDuration(10.0),
        transSplineLambda(0.0),
        rotSplineLambda(0.0),
        splineKnotsPerSecond(5),
        transSplineOrder(4),
        rotSplineOrder(4),
        linearVelocityTolerance(1.0),
        dmiPercentError(0.1),
        dmiVariance(1.0),
        flwPercentError(0.1),
        flwVariance(1e-6),
        frwPercentError(0.1),
        frwVariance(1e-6),
        rlwPercentError(0.1),
        rlwVariance(1e-6),
        rrwPercentError(0.1),
        rrwVariance(1e-6),
        steeringVariance(1e-6),
        wheelSpeedSensorCutoff(350),
        vyVariance(1e-1),
        vzVariance(1e-1),
        verbose(true),
        usePose(true),
        useVelocities(false),
        delayBound(50000000) {
    }

    CarCalibratorOptions::CarCalibratorOptions(const PropertyTree& config) {
      windowDuration = config.getDouble("windowDuration");
      verbose = config.getBool("verbose");
      usePose = config.getBool("usePose");
      useVelocities = config.getBool("useVelocities");
      delayBound = config.getInt("odometry/timeDelays/delayBound");

      transSplineLambda = config.getDouble("splines/transSplineLambda");
      rotSplineLambda = config.getDouble("splines/rotSplineLambda");
      splineKnotsPerSecond = config.getInt("splines/splineKnotsPerSecond");
      transSplineOrder = config.getInt("splines/transSplineOrder",
        transSplineOrder);
      rotSplineOrder = config.getInt("splines/rotSplineOrder");

      linearVelocityTolerance = config.getDouble(
        "odometry/sensors/linearVelocityTolerance");

      dmiPercentError = config.getDouble(
        "odometry/sensors/dmi/noise/dmiPercentError");
      dmiVariance = config.getDouble("odometry/sensors/dmi/noise/dmiVariance");
      flwPercentError = config.getDouble(
        "odometry/sensors/fws/noise/flwPercentError");
      flwVariance = config.getDouble("odometry/sensors/fws/noise/flwVariance");
      frwPercentError = config.getDouble(
        "odometry/sensors/fws/noise/frwPercentError");
      frwVariance = config.getDouble("odometry/sensors/fws/noise/frwVariance");
      rlwPercentError = config.getDouble(
        "odometry/sensors/rws/noise/rlwPercentError");
      rlwVariance = config.getDouble("odometry/sensors/rws/noise/rlwVariance");
      rrwPercentError = config.getDouble(
        "odometry/sensors/rws/noise/rrwPercentError");
      rrwVariance = config.getDouble("odometry/sensors/rws/noise/rrwVariance");
      steeringVariance = config.getDouble(
        "odometry/sensors/st/noise/steeringVariance");
      vyVariance = config.getDouble(
        "odometry/constraints/noise/vyVariance");
      vzVariance = config.getDouble(
        "odometry/constraints/noise/vzVariance");

      wheelSpeedSensorCutoff = config.getInt(
        "odometry/sensors/wheelSpeedSensorCutoff");
    }

  }
}
