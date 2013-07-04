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

#include "aslam/calibration/car/OptimizationProblemSpline.h"

#include <sm/boost/null_deleter.hpp>

#include <aslam/splines/BSplinePoseDesignVariable.hpp>

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

    const OptimizationProblemSpline::BSplinePoseDesignVariablesSP&
        OptimizationProblemSpline::getBSplinePoseDesignVariables() const {
      return _bsplinePoseDesignVariables;
    }

    size_t OptimizationProblemSpline::getNumBSplinePoseDesignVariables() const {
      return _bsplinePoseDesignVariables.size();
    }

    const aslam::splines::BSplinePoseDesignVariable*
        OptimizationProblemSpline::getBSplinePoseDesignVariable(size_t idx)
        const {
      if (idx >= _bsplinePoseDesignVariables.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getBSplinePoseDesignVariable(): "
          "index out of bounds", __FILE__, __LINE__);
      return _bsplinePoseDesignVariables[idx].get();
    }

    aslam::splines::BSplinePoseDesignVariable*
        OptimizationProblemSpline::getBSplinePoseDesignVariable(size_t idx) {
      if (idx >= _bsplinePoseDesignVariables.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblemSpline::getBSplinePoseDesignVariable(): "
          "index out of bounds", __FILE__, __LINE__);
      return _bsplinePoseDesignVariables[idx].get();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OptimizationProblemSpline::addDesignVariable(const
        BSplinePoseDesignVariableSP& bspdv, size_t groupId) {
      for (size_t i = 0; i < bspdv->numDesignVariables(); ++i)
        addDesignVariable(boost::shared_ptr<DesignVariable>(
          bspdv->designVariable(i), sm::null_deleter()), groupId);
      _bsplinePoseDesignVariables.push_back(bspdv);
    }

  }
}
