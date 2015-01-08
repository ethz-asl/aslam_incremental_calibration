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

/** \file DesignVariables.h
    \brief This file defines the DesignVariables structure which holds the
           design variables used during calibration.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_DESIGN_VARIABLES_H
#define ASLAM_CALIBRATION_EGOMOTION_DESIGN_VARIABLES_H

#include <cstdint>

#include <unordered_map>
#include <tuple>

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
    template <typename S> class GenericScalar;
    template <typename I, std::uintmax_t D> class FixedPointNumber;

  }
  namespace calibration {

    class OptimizationProblem;

    /** The structure DesignVariables holds the design variables used during
        calibration.
        \brief Calibration design variables.
      */
    struct DesignVariables :
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
      /// Batch shared pointer
      typedef boost::shared_ptr<OptimizationProblem> BatchSP;
      /// Time type
      typedef aslam::backend::FixedPointNumber<sm::timing::NsecTime,
        static_cast<long>(1e9)> Time;
      /// Time design variable
      typedef aslam::backend::GenericScalar<Time> TimeDesignVariable;
      /// Shared pointer to TimeDesignVariable
      typedef boost::shared_ptr<TimeDesignVariable> TimeDesignVariableSP;
      /// Calibration variables per sensor
      typedef std::tuple<TimeDesignVariableSP, EuclideanPointSP,
        RotationQuaternionSP> CalibrationVariables;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs design variables from property tree
      DesignVariables(const sm::PropertyTree& config);
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Adds the design variables to the batch
      void addToBatch(const BatchSP& batch, size_t groupId);
      /// Write current values to Eigen vector
      Eigen::VectorXd getParameters(size_t idx) const;
      /** @}
        */

      /** \name Public members
        @{
        */
      /// Calibration variables
      std::unordered_map<size_t, CalibrationVariables> calibrationVariables_;
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

#endif // ASLAM_CALIBRATION_EGOMOTION_DESIGN_VARIABLES_H
