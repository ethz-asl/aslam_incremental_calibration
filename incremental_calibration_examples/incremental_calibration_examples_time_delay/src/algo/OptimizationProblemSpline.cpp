/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/time-delay/algo/OptimizationProblemSpline.h"

#include <sm/boost/null_deleter.hpp>

#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/calibration/exceptions/OutOfBoundException.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    OptimizationProblemSpline::OptimizationProblemSpline() {
    }

    OptimizationProblemSpline::~OptimizationProblemSpline() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const OptimizationProblemSpline::TranslationSplinesSP&
        OptimizationProblemSpline::getTranslationSplines() const {
      return _translationSplines;
    }

    size_t OptimizationProblemSpline::getNumTranslationSplines() const {
      return _translationSplines.size();
    }

    const OptimizationProblemSpline::TranslationSpline*
        OptimizationProblemSpline::getTranslationSpline(size_t idx) const {
      if (idx >= _translationSplines.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getTranslationSpline(): "
          "index out of bounds", __FILE__, __LINE__);
      return _translationSplines[idx].get();
    }

    OptimizationProblemSpline::TranslationSpline*
        OptimizationProblemSpline::getTranslationSpline(size_t idx) {
      if (idx >= _translationSplines.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getTranslationSpline(): "
          "index out of bounds", __FILE__, __LINE__);
      return _translationSplines[idx].get();
    }

    const OptimizationProblemSpline::RotationSplinesSP&
        OptimizationProblemSpline::getRotationSplines() const {
      return _rotationSplines;
    }

    size_t OptimizationProblemSpline::getNumRotationSplines() const {
      return _rotationSplines.size();
    }

    const OptimizationProblemSpline::RotationSpline*
        OptimizationProblemSpline::getRotationSpline(size_t idx) const {
      if (idx >= _rotationSplines.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getRotationSpline(): "
          "index out of bounds", __FILE__, __LINE__);
      return _rotationSplines[idx].get();
    }

    OptimizationProblemSpline::RotationSpline*
        OptimizationProblemSpline::getRotationSpline(size_t idx) {
      if (idx >= _rotationSplines.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getRotationSpline(): "
          "index out of bounds", __FILE__, __LINE__);
      return _rotationSplines[idx].get();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OptimizationProblemSpline::addSpline(const TranslationSplineSP& spline,
        size_t groupId ) {
      const size_t numDV = spline->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        spline->designVariable(i)->setActive(true);
        addDesignVariable(boost::shared_ptr<DesignVariable>(
          spline->designVariable(i), sm::null_deleter()), groupId);
      }
      _translationSplines.push_back(spline);
    }

    void OptimizationProblemSpline::addSpline(const RotationSplineSP& spline,
        size_t groupId) {
      const size_t numDV = spline->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        spline->designVariable(i)->setActive(true);
        addDesignVariable(boost::shared_ptr<DesignVariable>(
          spline->designVariable(i), sm::null_deleter()), groupId);
      }
      _rotationSplines.push_back(spline);
    }

  }
}
