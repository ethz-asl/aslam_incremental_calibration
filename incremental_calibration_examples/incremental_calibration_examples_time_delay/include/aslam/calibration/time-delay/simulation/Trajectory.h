/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file Trajectory.h
    \brief This file represents a trajectory.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_H
#define ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_H

#include <vector>
#include <fstream>

#include <Eigen/Core>

#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {

    /** The structure Trajectory holds a trajectory.
        \brief Trajectory
      */
    struct Trajectory {
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \endcond

      /** \name Members
        @{
        */
      /// Vector of transformations
      std::vector<sm::kinematics::Transformation> w_T_v;
      /// Vector of body linear velocities
      std::vector<Eigen::Vector3d> v_v_wv;
      /// Vector of body angular velocities
      std::vector<Eigen::Vector3d> v_om_wv;
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Output w_T_v
      void w_T_vWrite(std::ofstream& stream) const;
      /// Output v_v_wv
      void v_v_wvWrite(std::ofstream& stream) const;
      /// Output v_om_wv
      void v_om_wvWrite(std::ofstream& stream) const;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_TIME_DELAY_TRAJECTORY_H
