/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/time-delay/simulation/SimulationParams.h"

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

    SimulationParams::SimulationParams() :
        v_T_p(rt2Transform(Eigen::Matrix3d::Identity(),
          Eigen::Vector3d(0.0, 0.0, 0.0))),
        b(0.8),
        k_l(360.0),
        t_l(0.1),
        k_r(360.0),
        t_r(0.1),
        dt(0.1),
        T(1000),
        sigma2_l(1e-3),
        sigma2_r(1e-3),
        sigma2_w_r_wp(Eigen::Matrix3d::Identity() * 1e-6),
        sigma2_w_R_p(Eigen::Matrix3d::Identity() * 1e-6) {
    }

    SimulationParams::SimulationParams(const PropertyTree& config) {
      b = config.getDouble("calibration/odometry/wss/b");
      k_l = config.getDouble("calibration/odometry/wss/k_l");
      t_l = config.getDouble("calibration/odometry/wss/t_l");
      k_r = config.getDouble("calibration/odometry/wss/k_r");
      t_r = config.getDouble("calibration/odometry/wss/t_r");
      const EulerAnglesYawPitchRoll ypr;
      v_T_p = rt2Transform(ypr.parametersToRotationMatrix(
        Eigen::Vector3d(
        config.getDouble("calibration/v_T_p/v_R_p/e1"),
        config.getDouble("calibration/v_T_p/v_R_p/e2"),
        config.getDouble("calibration/v_T_p/v_R_p/e3"))),
        Eigen::Vector3d(
        config.getDouble("calibration/v_T_p/v_r_vp/e1"),
        config.getDouble("calibration/v_T_p/v_r_vp/e2"),
        config.getDouble("calibration/v_T_p/v_r_vp/e3")));

      dt = config.getDouble("dt");
      T = config.getDouble("T");

      sigma2_l = config.getDouble("noise/odometry/wss/sigma2_l");
      sigma2_r = config.getDouble("noise/odometry/wss/sigma2_r");
      sigma2_w_r_wp = Eigen::Matrix3d::Identity() * Eigen::Vector3d(
        config.getDouble("noise/pose/sigma2_w_r_wp/e1"),
        config.getDouble("noise/pose/sigma2_w_r_wp/e2"),
        config.getDouble("noise/pose/sigma2_w_r_wp/e3")).asDiagonal();
      sigma2_w_R_p = Eigen::Matrix3d::Identity() * Eigen::Vector3d(
        config.getDouble("noise/pose/sigma2_w_R_p/e1"),
        config.getDouble("noise/pose/sigma2_w_R_p/e2"),
        config.getDouble("noise/pose/sigma2_w_R_p/e3")).asDiagonal();

      trajectoryParams = TrajectoryParams(sm::PropertyTree(config,
        "trajectory"));

    }

  }
}
