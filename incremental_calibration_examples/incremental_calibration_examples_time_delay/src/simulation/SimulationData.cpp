/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
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
