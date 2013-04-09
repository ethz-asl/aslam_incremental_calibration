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

#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/Optimizer2.hpp>

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"

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
        _options(options) {
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

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double IncrementalEstimator::optimize() {
      // optimization options
      aslam::backend::Optimizer2Options options;
      options.verbose = _options._verbose;
      options.doLevenbergMarquardt = false;
      options.linearSolver = "sparse_qr";

      // linear solver options
      aslam::backend::SparseQRLinearSolverOptions linearSolverOptions;
      linearSolverOptions.colNorm = _options._colNorm;
      linearSolverOptions.qrTol = _options._qrTol;

      // create optimizer with given options
      aslam::backend::Optimizer2 optimizer(options);

      // set options to the linear solver
      optimizer.getSolver<LinearSolver>()->setOptions(linearSolverOptions);

      // set the problem to the optimizer and optimize
      optimizer.setProblem(_problem);
      optimizer.optimize();
      return optimizer.getSolver<LinearSolver>()->computeSumLogDiagR(
        _problem->getGroupDim(_margGroupId));
    }

    void IncrementalEstimator::addBatch(const Batch& problem, bool force) {
      // insert new batch in the problem
      _problem->add(problem);

      // optimize
      const double sumLogDiagR = optimize();

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
      _problem->remove(idx);
      _sumLogDiagR = optimize();
    }

    Eigen::MatrixXd IncrementalEstimator::getMarginalizedCovariance() const {
      const size_t dim = _problem->getGroupDim(_margGroupId);
      return Eigen::MatrixXd::Zero(dim, dim);
    }

  }
}
