/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
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

/** \file OdometryDesignVariables.h
    \brief This file defines the OdometryDesignVariables structure which
           holds the design variables used during odometry calibration.
  */

#ifndef ASLAM_CALIBRATION_ODOMETRY_DESIGN_VARIABLES_H
#define ASLAM_CALIBRATION_ODOMETRY_DESIGN_VARIABLES_H

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <aslam/calibration/base/Serializable.h>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class EuclideanPoint;
    class RotationQuaternion;
    class Scalar;

  }
  namespace calibration {

    template <int M> class VectorDesignVariable;
    class OptimizationProblem;

    /** The structure OdometryDesignVariables holds the design variables
        used during odometry calibration.
        \brief Odometry calibration design variables.
      */
    struct OdometryDesignVariables :
      public virtual Serializable {
      /** \name Types definitions
        @{
        */
      /// Shared pointer to Euclidean point
      typedef boost::shared_ptr<aslam::backend::EuclideanPoint>
        EuclideanPointSP;
      /// Shared pointer to rotation quaternion
      typedef boost::shared_ptr<aslam::backend::RotationQuaternion>
        RotationQuaternionSP;
      /// Shared pointer to the steering coefficients
      typedef boost::shared_ptr<VectorDesignVariable<4> >
        SteeringCoefficientsSP;
      /// Shared pointer to the scalar
      typedef boost::shared_ptr<aslam::backend::Scalar> ScalarSP;
      /// Batch shared pointer
      typedef boost::shared_ptr<OptimizationProblem> BatchSP;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      OdometryDesignVariables();
      /// Constructs design variables from property tree
      OdometryDesignVariables(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds the odometry design variables to the batch
      void addToBatch(const BatchSP& batch, size_t groupId);
      /// Write current values to Eigen vector
      Eigen::VectorXd getParameters() const;
      /** @}
        */

      /** \name Public members
        @{
        */
      /// Relative position of the Applanix ref. w.r. to vehicle frame in F_v
      EuclideanPointSP v_r_vr;
      /// Relative orientation of the Applanix ref. w.r. to vehicle frame in F_v
      RotationQuaternionSP v_R_r;
      /// Steering wheel coefficients
      SteeringCoefficientsSP a;
      /// Half rear axle track
      ScalarSP e_r;
      /// Half front axle track
      ScalarSP e_f;
      /// Wheel base
      ScalarSP L;
      /// Rear left wheel scale factor
      ScalarSP k_rl;
      /// Rear right wheel scale factor
      ScalarSP k_rr;
      /// Front left wheel scale factor
      ScalarSP k_fl;
      /// Front right wheel scale factor
      ScalarSP k_fr;
      /// DMI scale factor
      ScalarSP k_dmi;
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_ODOMETRY_DESIGN_VARIABLES_H
