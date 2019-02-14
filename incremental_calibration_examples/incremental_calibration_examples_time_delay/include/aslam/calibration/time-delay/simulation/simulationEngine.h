/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file simulationEngine.h
    \brief This file performs the simulation.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_ENGINE_H
#define ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_ENGINE_H

#include <Eigen/Core>

namespace sm {
  namespace kinematics {

    class Transformation;

  }
}
namespace aslam {
  namespace calibration {

    class SimulationData;
    class SimulationParams;

    /** \name Methods
      @{
      */
    /// Simulate
    void simulate(const SimulationParams& params, SimulationData& data);
    /// Integrate a discrete-time 3d motion model
    sm::kinematics::Transformation integrateMotionModel(const
      sm::kinematics::Transformation& w_T_v_km1, const Eigen::Vector3d&
      v_v_wv_km1, const Eigen::Vector3d& v_om_wv_km1, double dt);
    /// Generate 2d body linear and angular velocities along a sine wave
    Eigen::Vector2d genSineBodyVel2d(double& w_phi_v_km1, Eigen::Vector2d&
      w_r_wv_km1, double t, double dt, double A, double f);
    /// Generate wheel linear velocity from body linear/angular velocity
    Eigen::Vector3d genWheelVelocity(const Eigen::Vector3d& v_v_wv, const
      Eigen::Vector3d& v_om_wv, const Eigen::Vector3d& v_r_vc);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_SIMULATION_ENGINE_H
