/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/time-delay/simulation/TrajectoryParams.h"

#include <sm/PropertyTree.hpp>

using namespace sm;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    TrajectoryParams::TrajectoryParams() :
        w_T_v_0(),
        A(15.0),
        f(0.01) {
    }

    TrajectoryParams::TrajectoryParams(const PropertyTree& config) {
      A = config.getDouble("A");
      f = config.getDouble("f");
    }

  }
}
