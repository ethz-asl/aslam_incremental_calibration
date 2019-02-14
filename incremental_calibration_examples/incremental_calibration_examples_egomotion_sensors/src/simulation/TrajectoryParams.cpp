/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/egomotion/simulation/TrajectoryParams.h"

#include <sm/PropertyTree.hpp>

#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace sm;
using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    TrajectoryParams::TrajectoryParams() :
        w_T_v_0(),
        A(15.0),
        f(0.01),
        transSplineLambda(0.0),
        rotSplineLambda(0.0),
        splineKnotsPerSecond(5),
        transSplineOrder(4),
        rotSplineOrder(4) {
    }

    TrajectoryParams::TrajectoryParams(const PropertyTree& config) {
      A = config.getDouble("A");
      f = config.getDouble("f");
      const EulerAnglesYawPitchRoll ypr;
      w_T_v_0 = rt2Transform(ypr.parametersToRotationMatrix(Eigen::Vector3d(
        config.getDouble("w_T_v_0/yaw"), config.getDouble("w_T_v_0/pitch"),
        config.getDouble("w_T_v_0/roll"))), Eigen::Vector3d(
        config.getDouble("w_T_v_0/x"), config.getDouble("w_T_v_0/y"),
        config.getDouble("w_T_v_0/z")));
      transSplineLambda = config.getDouble("splines/transSplineLambda");
      rotSplineLambda = config.getDouble("splines/rotSplineLambda");
      splineKnotsPerSecond = config.getInt("splines/splineKnotsPerSecond");
      transSplineOrder = config.getInt("splines/transSplineOrder",
        transSplineOrder);
      rotSplineOrder = config.getInt("splines/rotSplineOrder");
      type = config.getString("type");
    }

  }
}
