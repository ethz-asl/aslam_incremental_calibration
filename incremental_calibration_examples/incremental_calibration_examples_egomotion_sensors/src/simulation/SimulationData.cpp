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

#include "aslam/calibration/time-delay/simulation/SimulationData.h"

#include <algorithm>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Streaming methods                                                          */
/******************************************************************************/

    void SimulationData::w_v_wwlWrite(std::ofstream& stream) const {
      std::for_each(w_v_wwl.cbegin(), w_v_wwl.cend(), [&](decltype(
        *w_v_wwl.cbegin()) x){stream << x.transpose() << std::endl;});
    }

    void SimulationData::w_v_wwrWrite(std::ofstream& stream) const {
      std::for_each(w_v_wwr.cbegin(), w_v_wwr.cend(), [&](decltype(
        *w_v_wwr.cbegin()) x){stream << x.transpose() << std::endl;});
    }

    void SimulationData::rwDataWrite(std::ofstream& stream) const {
      std::for_each(rwData.cbegin(), rwData.cend(), [&](decltype(
        *rwData.cbegin()) x){stream << x.second.value << std::endl;});
    }

    void SimulationData::rwData_nWrite(std::ofstream& stream) const {
      std::for_each(rwData_n.cbegin(), rwData_n.cend(), [&](decltype(
        *rwData_n.cbegin()) x){stream << x.second.value << std::endl;});
    }

    void SimulationData::lwDataWrite(std::ofstream& stream) const {
      std::for_each(lwData.cbegin(), lwData.cend(), [&](decltype(
        *lwData.cbegin()) x){stream << x.second.value << std::endl;});
    }

    void SimulationData::lwData_nWrite(std::ofstream& stream) const {
      std::for_each(lwData_n.cbegin(), lwData_n.cend(), [&](decltype(
        *lwData_n.cbegin()) x){stream << x.second.value << std::endl;});
    }

    void SimulationData::poseDataWrite(std::ofstream& stream) const {
      std::for_each(poseData.cbegin(), poseData.cend(), [&](decltype(
        *poseData.cbegin()) x){stream << x.second.w_r_wp.transpose()
        << " " << x.second.w_R_p.transpose() << std::endl;});
    }

    void SimulationData::poseData_nWrite(std::ofstream& stream) const {
      std::for_each(poseData_n.cbegin(), poseData_n.cend(), [&](decltype(
        *poseData_n.cbegin()) x){stream << x.second.w_r_wp.transpose()
        << " " << x.second.w_R_p.transpose() << std::endl;});
    }

  }
}
