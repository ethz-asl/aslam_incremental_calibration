/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file OptimizationProblemSpline.h
    \brief This file defines the OptimizationProblemSpline class, which is
           a specialization of OptimizationProblem for handling splines.
  */

#ifndef ASLAM_CALIBRATION_EGOMOTION_OPTIMIZATION_PROBLEM_SPLINE_H
#define ASLAM_CALIBRATION_EGOMOTION_OPTIMIZATION_PROBLEM_SPLINE_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/exceptions/OutOfBoundException.h>

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
      typedef aslam::splines::OPTBSpline<
        bsplines::UnitQuaternionBSpline<Eigen::Dynamic,
        bsplines::NsecTimePolicy>::CONF>::BSpline RotationSpline;
      /// Rotation spline shared pointer
      typedef boost::shared_ptr<RotationSpline> RotationSplineSP;
      /// Rotation splines container
      typedef std::vector<RotationSplineSP> RotationSplinesSP;
      /// Translation spline
      typedef aslam::splines::OPTBSpline<
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
      OptimizationProblemSpline() = default;
      /// Copy constructor
      OptimizationProblemSpline(const Self& other) = delete;
      /// Copy assignment operator
      OptimizationProblemSpline& operator = (const Self& other) = delete;
      /// Move constructor
      OptimizationProblemSpline(Self&& other) = delete;
      /// Move assignment operator
      OptimizationProblemSpline& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~OptimizationProblemSpline() = default;
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
      const TranslationSplinesSP& getTranslationSplines() const {
        return translationSplines_;
      }
      /// Returns the number of translation spliens
      size_t getNumTranslationSplines() const {
        return translationSplines_.size();
      }
      /// Returns a translation spline by index
      const TranslationSpline* getTranslationSpline(size_t idx = 0) const {
#ifndef NDEBUG
        if (idx >= translationSplines_.size())
          throw OutOfBoundException<size_t>(idx,
            "OptimizationProblemSpline::getTranslationSpline(): "
            "index out of bounds", __FILE__, __LINE__);
#endif
        return translationSplines_[idx].get();
      }
      /// Returns a translation spline by index
      TranslationSpline* getTranslationSpline(size_t idx = 0) {
#ifndef NDEBUG
        if (idx >= translationSplines_.size())
          throw OutOfBoundException<size_t>(idx,
            "OptimizationProblemSpline::getTranslationSpline(): "
            "index out of bounds", __FILE__, __LINE__);
#endif
        return translationSplines_[idx].get();
      }
      /// Returns the rotation splines
      const RotationSplinesSP& getRotationSplines() const {
        return rotationSplines_;
      }
      /// Returns the number of rotation splines
      size_t getNumRotationSplines() const {
        return rotationSplines_.size();
      }
      /// Returns a rotation spline by index
      const RotationSpline* getRotationSpline(size_t idx = 0) const {
#ifndef NDEBUG
        if (idx >= rotationSplines_.size())
          throw OutOfBoundException<size_t>(idx,
            "OptimizationProblemSpline::getRotationSpline(): "
            "index out of bounds", __FILE__, __LINE__);
#endif
        return rotationSplines_[idx].get();
      }
      /// Returns a rotation spline by index
      RotationSpline* getRotationSpline(size_t idx = 0) {
#ifndef NDEBUG
        if (idx >= rotationSplines_.size())
          throw OutOfBoundException<size_t>(idx,
            "OptimizationProblemSpline::getRotationSpline(): "
            "index out of bounds", __FILE__, __LINE__);
#endif
        return rotationSplines_[idx].get();
      }
      /** @}
        */

    protected:
      /** \name Protected members
        @{
        */
      /// Storage for the translation splines
      TranslationSplinesSP translationSplines_;
      /// Storage for the rotation splines
      RotationSplinesSP rotationSplines_;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_EGOMOTION_OPTIMIZATION_PROBLEM_SPLINE_H
