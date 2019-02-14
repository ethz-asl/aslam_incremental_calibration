/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

#include "aslam/calibration/car/algo/splinesToFile.h"

#include <algorithm>

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>

#include <bsplines/NsecTimePolicy.hpp>

#include "aslam/calibration/car/algo/OptimizationProblemSpline.h"

using namespace sm::timing;
using namespace sm::kinematics;

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void writeSplines(const IncrementalEstimatorSP& estimator, double dt,
        std::ofstream& stream) {
      auto batches = estimator->getProblem()->getOptimizationProblems();
      for (auto it = batches.cbegin(); it != batches.cend(); ++it) {
        auto transSplines = dynamic_cast<const OptimizationProblemSpline*>(
          it->get())->getTranslationSplines();
        auto rotSplines = dynamic_cast<const OptimizationProblemSpline*>(
          it->get())->getRotationSplines();
        for (auto itSplines = transSplines.cbegin(); itSplines !=
            transSplines.cend(); ++itSplines) {
          writeSplines(*itSplines, rotSplines.at(std::distance(
            transSplines.cbegin(), itSplines)), dt, stream);
        }
      }
    }

    void writeSplines(const TranslationSplineSP& transSpline, const
        RotationSplineSP& rotSpline, double dt, std::ofstream& stream) {
      auto t = transSpline->getMinTime();
      auto T = transSpline->getMaxTime();
      const EulerAnglesYawPitchRoll ypr;
      while (t < T) {
        auto translationExpressionFactory =
          transSpline->getExpressionFactoryAt<0>(t);
        stream << translationExpressionFactory.getValueExpression().toValue().
            transpose() << " ";
        auto rotationExpressionFactory =
          rotSpline->getExpressionFactoryAt<0>(t);
        stream << ypr.rotationMatrixToParameters(quat2r(
          rotationExpressionFactory.getValueExpression().toValue())).
            transpose() << std::endl;
        t += secToNsec(dt);
      }
    }

  }
}
