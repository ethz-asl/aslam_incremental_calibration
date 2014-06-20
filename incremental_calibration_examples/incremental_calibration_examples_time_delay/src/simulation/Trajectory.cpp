/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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
