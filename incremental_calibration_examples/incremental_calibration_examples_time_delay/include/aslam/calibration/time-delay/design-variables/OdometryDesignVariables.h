/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file OdometryDesignVariables.h
    \brief This file defines the OdometryDesignVariables structure which
           holds the design variables used during odometry calibration.
  */

#ifndef ASLAM_CALIBRATION_TIME_DELAY_ODOMETRY_DESIGN_VARIABLES_H
#define ASLAM_CALIBRATION_TIME_DELAY_ODOMETRY_DESIGN_VARIABLES_H

#include <cstdint>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <sm/timing/NsecTimeUtilities.hpp>

#include <aslam/calibration/base/Serializable.h>

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class EuclideanPoint;
    class RotationQuaternion;
    class Scalar;
    template <typename S> class GenericScalar;
    template <typename I, std::uintmax_t D> class FixedPointNumber;

  }
  namespace calibration {

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
      /// Shared pointer to the scalar
      typedef boost::shared_ptr<aslam::backend::Scalar> ScalarSP;
      /// Batch shared pointer
      typedef boost::shared_ptr<OptimizationProblem> BatchSP;
      /// Time type
      typedef aslam::backend::FixedPointNumber<sm::timing::NsecTime, (long)1e9>
        Time;
      /// Time design variable
      typedef aslam::backend::GenericScalar<Time> TimeDesignVariable;
      /// Shared pointer to TimeDesignVariable
      typedef boost::shared_ptr<TimeDesignVariable> TimeDesignVariableSP;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
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
      /// Relative position of the pose sensor w.r.t. vehicle frame in F_v
      EuclideanPointSP v_r_vp;
      /// Relative orientation of the pose sensor w.r.t. vehicle frame in F_v
      RotationQuaternionSP v_R_p;
      /// Half wheel base
      ScalarSP b;
      /// Left wheel scale factor
      ScalarSP k_l;
      /// Right wheel scale factor
      ScalarSP k_r;
      /// Time delay for left wheel
      TimeDesignVariableSP t_l;
      /// Time delay for right wheel
      TimeDesignVariableSP t_r;
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

#endif // ASLAM_CALIBRATION_TIME_DELAY_ODOMETRY_DESIGN_VARIABLES_H
