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

#include "aslam/calibration/egomotion/simulation/SimulationParams.h"

#include <cmath>

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

    SimulationParams::SimulationParams(const PropertyTree& config) {
//      
//      b = config.getDouble("calibration/odometry/wss/b");
//      k_l = config.getDouble("calibration/odometry/wss/k_l");
//      t_l = config.getDouble("calibration/odometry/wss/t_l");
//      k_r = config.getDouble("calibration/odometry/wss/k_r");
//      t_r = config.getDouble("calibration/odometry/wss/t_r");
//      const EulerAnglesYawPitchRoll ypr;
//      v_T_p = rt2Transform(ypr.parametersToRotationMatrix(
//        Eigen::Vector3d(
//        config.getDouble("calibration/v_T_p/v_R_p/e1"),
//        config.getDouble("calibration/v_T_p/v_R_p/e2"),
//        config.getDouble("calibration/v_T_p/v_R_p/e3"))),
//        Eigen::Vector3d(
//        config.getDouble("calibration/v_T_p/v_r_vp/e1"),
//        config.getDouble("calibration/v_T_p/v_r_vp/e2"),
//        config.getDouble("calibration/v_T_p/v_r_vp/e3")));

      dt = config.getDouble("dt");
      T = config.getDouble("T");

//      sigma2_l = config.getDouble("noise/odometry/wss/sigma2_l");
//      sigma2_r = config.getDouble("noise/odometry/wss/sigma2_r");
//      sigma2_w_r_wp = Eigen::Matrix3d::Identity() * Eigen::Vector3d(
//        config.getDouble("noise/pose/sigma2_w_r_wp/e1"),
//        config.getDouble("noise/pose/sigma2_w_r_wp/e2"),
//        config.getDouble("noise/pose/sigma2_w_r_wp/e3")).asDiagonal();
//      sigma2_w_R_p = Eigen::Matrix3d::Identity() * Eigen::Vector3d(
//        config.getDouble("noise/pose/sigma2_w_R_p/e1"),
//        config.getDouble("noise/pose/sigma2_w_R_p/e2"),
//        config.getDouble("noise/pose/sigma2_w_R_p/e3")).asDiagonal();

      trajectoryParams = TrajectoryParams(sm::PropertyTree(config,
        "trajectory"));

    }

  }
}
