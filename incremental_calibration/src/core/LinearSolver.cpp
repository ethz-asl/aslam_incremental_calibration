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

#include "aslam/calibration/core/LinearSolver.h"

#include <sm/PropertyTree.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LinearSolver::LinearSolver(const Options& options) :
        _options(options),
        _factor(NULL) {
      cholmod_l_start(&_cholmod);
    }

    LinearSolver::LinearSolver(const sm::PropertyTree& config) :
        _factor(NULL) {
      cholmod_l_start(&_cholmod);
    }

    LinearSolver::~LinearSolver() {
      if (_factor)
        SuiteSparseQR_free(&_factor, &_cholmod);
      cholmod_l_finish(&_cholmod);
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const LinearSolver::Options& LinearSolver::getOptions() const {
      return _options;
    }

    LinearSolver::Options& LinearSolver::getOptions() {
      return _options;
    }

    std::string LinearSolver::name() const {
      return std::string("marginal_spqr");
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void LinearSolver::buildSystem(size_t numThreads, bool useMEstimator) {
    }

    bool LinearSolver::solveSystem(Eigen::VectorXd& x) {
    }

    double LinearSolver::rhsJtJrhs() {
    }

    void LinearSolver::initMatrixStructureImplementation(const
        std::vector<aslam::backend::DesignVariable*>& dvs, const
        std::vector<aslam::backend::ErrorTerm*>& errors, bool
        useDiagonalConditioner) {
    }

  }
}
