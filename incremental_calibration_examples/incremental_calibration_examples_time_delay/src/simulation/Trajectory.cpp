/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/time-delay/simulation/Trajectory.h"

#include <algorithm>

#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Streaming methods                                                          */
/******************************************************************************/

    void Trajectory::w_T_vWrite(std::ofstream& stream) const {
      const EulerAnglesYawPitchRoll ypr;
      std::for_each(w_T_v.cbegin(), w_T_v.cend(), [&](decltype(
        *w_T_v.cbegin()) x){stream << x.t().transpose() << " " <<
        ypr.rotationMatrixToParameters(x.C()).transpose() << std::endl;});
    }

    void Trajectory::v_v_wvWrite(std::ofstream& stream) const {
      std::for_each(v_v_wv.cbegin(), v_v_wv.cend(), [&](decltype(
        *v_v_wv.cbegin()) x){stream << x.transpose() << std::endl;});
    }

    void Trajectory::v_om_wvWrite(std::ofstream& stream) const {
      std::for_each(v_om_wv.cbegin(), v_om_wv.cend(), [&](decltype(
        *v_om_wv.cbegin()) x){stream << x.transpose() << std::endl;});
    }

  }
}
