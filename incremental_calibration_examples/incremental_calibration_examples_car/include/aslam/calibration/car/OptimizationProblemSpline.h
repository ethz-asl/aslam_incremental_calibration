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

/** \file OptimizationProblemSpline.h
    \brief This file defines the OptimizationProblemSpline class, which is
           a specialization of OptimizationProblem for handling splines.
  */

#ifndef ASLAM_CALIBRATION_CAR_OPTIMIZATION_PROBLEM_SPLINE_H
#define ASLAM_CALIBRATION_CAR_OPTIMIZATION_PROBLEM_SPLINE_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>

namespace aslam {
  namespace splines {

    class BSplinePoseDesignVariable;

  }
  namespace calibration {

    /** The class OptimizationProblemSpline is a specialization of
        OptimizationProblem for handling splines.
        \brief Optimization problem for splines.
      */
    class OptimizationProblemSpline :
      public OptimizationProblem {
    public:
      /** \name Types definitions
        @{
        */
      /// Shared pointer to B-spline pose design variable
      typedef boost::shared_ptr<aslam::splines::BSplinePoseDesignVariable>
        BSplinePoseDesignVariableSP;
      /// Container for B-spline pose design variables (shared pointer)
      typedef std::vector<BSplinePoseDesignVariableSP>
        BSplinePoseDesignVariablesSP;
      /// Self type
      typedef OptimizationProblemSpline Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      OptimizationProblemSpline();
      /// Copy constructor
      OptimizationProblemSpline(const Self& other) = delete;
      /// Copy assignment operator
      OptimizationProblemSpline& operator = (const Self& other) = delete;
      /// Move constructor
      OptimizationProblemSpline(Self&& other) = delete;
      /// Move assignment operator
      OptimizationProblemSpline& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~OptimizationProblemSpline();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Inserts a B-spline pose design variable into the problem
      void addDesignVariable(const BSplinePoseDesignVariableSP& bspdv,
        size_t groupId = 0);
      /// Using the functions from the base class
      using OptimizationProblem::addDesignVariable;
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the B-spline pose design variables
      const BSplinePoseDesignVariablesSP& getBSplinePoseDesignVariables() const;
      /// Returns the number of B-spline pose design variables
      size_t getNumBSplinePoseDesignVariables() const;
      /// Returns a B-spline by index
      const aslam::splines::BSplinePoseDesignVariable*
        getBSplinePoseDesignVariable(size_t idx) const;
      /// Returns a B-spline by index
      aslam::splines::BSplinePoseDesignVariable*
        getBSplinePoseDesignVariable(size_t idx);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Storage for the B-spline pose design variables shared pointers
      BSplinePoseDesignVariablesSP _bsplinePoseDesignVariables;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_OPTIMIZATION_PROBLEM_SPLINE_H
