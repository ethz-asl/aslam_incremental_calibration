/******************************************************************************
 * Copyright (C) 2015 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/egomotion/algo/OptimizationProblemSpline.h"

#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OptimizationProblemSpline::addSpline(const TranslationSplineSP& spline,
        size_t groupId ) {
      const auto numDV = spline->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        spline->designVariable(i)->setActive(true);
        addDesignVariable(boost::shared_ptr<DesignVariable>(
          spline->designVariable(i), sm::null_deleter()), groupId);
      }
      translationSplines_.push_back(spline);
    }

    void OptimizationProblemSpline::addSpline(const RotationSplineSP& spline,
        size_t groupId) {
      const auto numDV = spline->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        spline->designVariable(i)->setActive(true);
        addDesignVariable(boost::shared_ptr<DesignVariable>(
          spline->designVariable(i), sm::null_deleter()), groupId);
      }
      rotationSplines_.push_back(spline);
    }

  }
}
