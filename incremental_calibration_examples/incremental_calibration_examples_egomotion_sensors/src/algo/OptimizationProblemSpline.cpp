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
