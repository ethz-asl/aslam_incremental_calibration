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

#include "aslam/calibration/core/IncrementalEstimator.h"

#include <algorithm>
#include <utility>

#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/Optimizer2.hpp>

#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/algorithms/matrixOperations.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Static Members Initialization                                              */
/******************************************************************************/

const struct IncrementalEstimator::Options
  IncrementalEstimator::_defaultOptions = {0.5, 0.02, true, true};

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator(size_t groupId,
        const Options& options) :
        _problem(new IncrementalOptimizationProblem()),
        _margGroupId(groupId),
        _mi(0),
        _sumLogDiagR(0),
        _options(options),
        _optimizer(new Optimizer()) {
      // optimization options
      aslam::backend::Optimizer2Options& optOptions = _optimizer->options();
      optOptions.verbose = _options._verbose;
      optOptions.doLevenbergMarquardt = false;
      optOptions.linearSolver = "sparse_qr";
    }

    IncrementalEstimator::~IncrementalEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const IncrementalOptimizationProblem*
        IncrementalEstimator::getProblem() const {
      return _problem.get();
    }

    const IncrementalEstimator::Options&
        IncrementalEstimator::getOptions() const {
      return _options;
    }

    IncrementalEstimator::Options& IncrementalEstimator::getOptions() {
      return _options;
    }

    double IncrementalEstimator::getMutualInformation() const {
      return _mi;
    }

    size_t IncrementalEstimator::getMargGroupId() const {
      return _margGroupId;
    }

    const aslam::backend::CompressedColumnMatrix<ssize_t>&
        IncrementalEstimator::getJacobianTranspose() const {
      return _optimizer->getSolver<LinearSolver>()->getJacobianTranspose();
    }

    size_t IncrementalEstimator::getRank() const {
      return _optimizer->getSolver<LinearSolver>()->getRank();
    }

    double IncrementalEstimator::getQRTol() const {
      return _optimizer->getSolver<LinearSolver>()->getTol();
    }

    std::vector<ssize_t> IncrementalEstimator::getPermutationVector() const {
      return _optimizer->getSolver<LinearSolver>()->getPermutationVector();
    }

    const aslam::backend::CompressedColumnMatrix<ssize_t>&
        IncrementalEstimator::getR() const {
      return _optimizer->getSolver<LinearSolver>()->getR();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void IncrementalEstimator::optimize() {
      // linear solver options
      aslam::backend::SparseQRLinearSolverOptions linearSolverOptions;
      linearSolverOptions.colNorm = _options._colNorm;
      linearSolverOptions.qrTol = _options._qrTol;

      // reset the linear solver
      _optimizer->initializeLinearSolver();

      // set options to the linear solver
      _optimizer->getSolver<LinearSolver>()->setOptions(linearSolverOptions);

      // set the problem to the optimizer and optimize
      _optimizer->setProblem(_problem);
      _optimizer->optimize();
    }

    void IncrementalEstimator::addBatch(const BatchSP& problem, bool force) {
      // insert new batch in the problem
      _problem->add(problem);

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // optimize
      optimize();

      // compute the sum log diag R
      const double sumLogDiagR = getSumLogDiagR();

      // batch is kept?
      bool keepBatch = false;

      // first round of estimation?
      if (!_sumLogDiagR) {
        _sumLogDiagR = sumLogDiagR;
        keepBatch = true;
      }
      else {
        // compute MI
        const double mi = sumLogDiagR - _sumLogDiagR;

        // MI improvement
        if (mi > _options._miTol) {
          _sumLogDiagR = sumLogDiagR;
          keepBatch = true;
          _mi = mi;
        }
      }

      // remove batch if necessary
      if (!keepBatch && !force)
        _problem->remove(_problem->getNumOptimizationProblems() - 1);
    }

    void IncrementalEstimator::removeBatch(size_t idx) {
      // remove the batch
      _problem->remove(idx);

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // optimize back
      optimize();

      // update mutual information
      const double sumLogDiagR = getSumLogDiagR();
      _mi = sumLogDiagR - _sumLogDiagR;
      _sumLogDiagR = sumLogDiagR;
    }

    Eigen::MatrixXd IncrementalEstimator::getMarginalizedCovariance() const {
      const size_t dim = _problem->getGroupDim(_margGroupId);
      const aslam::backend::CompressedColumnMatrix<ssize_t>& R = getR();
      const size_t numCols = R.cols();
      return computeCovariance(R, numCols - dim, numCols - 1);
    }

    void IncrementalEstimator::orderMarginalizedDesignVariables() {
      auto groupsOrdering = _problem->getGroupsOrdering();
      auto margGroupIt = std::find(groupsOrdering.begin(),
        groupsOrdering.end(), _margGroupId);
      if (margGroupIt == groupsOrdering.end())
        throw InvalidOperationException(
          "IncrementalEstimator::orderMarginalizedDesignVariables(): "
          "marginalized group ID should appear in the problem");
      else {
        if (*margGroupIt != groupsOrdering.back()) {
          std::swap(*margGroupIt, groupsOrdering.back());
          _problem->setGroupsOrdering(groupsOrdering);
        }
      }
    }

    double IncrementalEstimator::getSumLogDiagR() const {
      const size_t dim = _problem->getGroupDim(_margGroupId);
      const aslam::backend::CompressedColumnMatrix<ssize_t>& R = getR();
      const size_t numCols = R.cols();
      return computeSumLogDiagR(R, numCols - dim, numCols - 1);
    }

  }
}
