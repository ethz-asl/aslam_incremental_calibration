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

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>

namespace bsplines {

  struct NsecTimePolicy;

}
namespace aslam {
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
      /// Rotation spline
      typedef typename aslam::splines::OPTBSpline<typename
        bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Rotation splines container
      typedef std::vector<RotationSplineSP> RotationSplinesSP;
      /// Translation spline
      typedef typename aslam::splines::OPTBSpline<typename
        bsplines::EuclideanBSpline<Eigen::Dynamic, 3,
        bsplines::NsecTimePolicy>::CONF>::BSpline TranslationSpline;
      /// Euclidean spline shared pointer
      typedef boost::shared_ptr<TranslationSpline> TranslationSplineSP;
      /// Translation splines container
      typedef std::vector<TranslationSplineSP> TranslationSplinesSP;
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
      /// Inserts a translation spline into the problem
      void addSpline(const TranslationSplineSP& spline, size_t groupId = 0);
      /// Inserts a rotation spline into the problem
      void addSpline(const RotationSplineSP& spline, size_t groupId = 0);
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the translation splines
      const TranslationSplinesSP& getTranslationSplines() const;
      /// Returns the number of translation spliens
      size_t getNumTranslationSplines() const;
      /// Returns a translation spline by index
      const TranslationSpline* getTranslationSpline(size_t idx = 0) const;
      /// Returns a translation spline by index
      TranslationSpline* getTranslationSpline(size_t idx = 0);
      /// Returns the rotation splines
      const RotationSplinesSP& getRotationSplines() const;
      /// Returns the number of rotation spliens
      size_t getNumRotationSplines() const;
      /// Returns a rotation spline by index
      const RotationSpline* getRotationSpline(size_t idx = 0) const;
      /// Returns a rotation spline by index
      RotationSpline* getRotationSpline(size_t idx = 0);
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// Storage for the translation splines
      TranslationSplinesSP _translationSplines;
      /// Storage for the rotation splines
      RotationSplinesSP _rotationSplines;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CAR_OPTIMIZATION_PROBLEM_SPLINE_H
