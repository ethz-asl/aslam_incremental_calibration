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
#include <vector>
#include <ostream>

#include <boost/make_shared.hpp>

#include <sm/PropertyTree.hpp>

#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/CompressedColumnMatrix.hpp>

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"
#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/algorithms/marginalize.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator(size_t groupId,
        const Options& options, const LinearSolverOptions&
        linearSolverOptions, const OptimizerOptions& optimizerOptions) :
        _problem(boost::make_shared<IncrementalOptimizationProblem>()),
        _margGroupId(groupId),
        _mi(0),
        _svLogSum(0),
        _options(options),
        _optimizer(boost::make_shared<Optimizer>(optimizerOptions)),
        _nRank(0),
        _qrTol(0) {
      // create linear solver and trust region policy for the optimizer
      OptimizerOptions& optOptions = _optimizer->options();
      optOptions.linearSystemSolver =
        boost::make_shared<LinearSolver>(linearSolverOptions);
      optOptions.trustRegionPolicy = boost::make_shared<TrustRegionPolicy>();
      _optimizer->initializeLinearSolver();
      _optimizer->initializeTrustRegionPolicy();

      // attach the problem to the optimizer
      _optimizer->setProblem(_problem);
    }

    IncrementalEstimator::IncrementalEstimator(const sm::PropertyTree& config) :
        _mi(0),
        _svLogSum(0),
        _nRank(0),
        _qrTol(0) {
      // create the optimizer, linear solver, and trust region policy
      _optimizer = boost::make_shared<Optimizer>(
        sm::PropertyTree(config, "optimizer"),
        boost::make_shared<LinearSolver>(
        sm::PropertyTree(config, "optimizer/linearSolver")),
        boost::make_shared<TrustRegionPolicy>());

      // create the problem and attach it to the optimizer
      _problem = boost::make_shared<IncrementalOptimizationProblem>();
      _optimizer->setProblem(_problem);

      // parse the options and set them
      _options._miTol = config.getDouble("miTol", _options._miTol);
      _options._normTol = config.getDouble("normTol", _options._normTol);
      _options._epsTolSVD = config.getDouble("epsTolSVD", _options._epsTolSVD);
      _options._verbose = config.getBool("verbose", _options._verbose);
      _margGroupId = config.getInt("groupId");
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

    const IncrementalEstimator::LinearSolverOptions&
        IncrementalEstimator::getLinearSolverOptions() const {
      return _optimizer->getSolver<LinearSolver>()->getOptions();
    }

    IncrementalEstimator::LinearSolverOptions&
        IncrementalEstimator::getLinearSolverOptions() {
      return _optimizer->getSolver<LinearSolver>()->getOptions();
    }

    const IncrementalEstimator::OptimizerOptions&
        IncrementalEstimator::getOptimizerOptions() const {
      return _optimizer->options();
    }

    IncrementalEstimator::OptimizerOptions&
        IncrementalEstimator::getOptimizerOptions() {
      return _optimizer->options();
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
      return _nRank;
    }

    size_t IncrementalEstimator::getRankDeficiency() const {
      return _optimizer->getSolver<LinearSolver>()->
        getJacobianTranspose().rows() - _nRank;
    }

    size_t IncrementalEstimator::getMarginalRank() const {
      return _CS.cols();
    }

    size_t IncrementalEstimator::getMarginalRankDeficiency() const {
      return _NS.cols();
    }

    double IncrementalEstimator::getQRTol() const {
      return _qrTol;
    }

    size_t IncrementalEstimator::getCholmodMemoryUsage() const {
      return _optimizer->getSolver<LinearSolver>()->getMemoryUsage();
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedNullSpace()
        const {
      return _NS;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedColumnSpace()
        const {
      return _CS;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedCovariance()
        const {
      return _Sigma;
    }

    const Eigen::MatrixXd&
        IncrementalEstimator::getProjectedMarginalizedCovariance() const {
      return _SigmaP;
    }

    const Eigen::MatrixXd&
        IncrementalEstimator::getMarginalizedInformationMatrix() const {
      return _Omega;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    IncrementalEstimator::ReturnValue IncrementalEstimator::reoptimize() {
      // query the time
      const double timeStart = Timestamp::now();

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // return value
      ReturnValue ret;

      // analyze marginalized system
      const size_t dim = _problem->getGroupDim(_margGroupId);
      const size_t numCols = getJacobianTranspose().rows();
      _svLogSum =  marginalize(getJacobianTranspose(), numCols - dim, ret._NS,
        ret._CS, ret._Sigma, ret._SigmaP, ret._Omega, _options._normTol,
        _options._epsTolSVD);
      _NS = ret._NS;
      _CS = ret._CS;
      _Sigma = ret._Sigma;
      _SigmaP = ret._SigmaP;
      _Omega = ret._Omega;
      _nRank = _optimizer->getSolver<LinearSolver>()->getRank();
      _qrTol = _optimizer->getSolver<LinearSolver>()->getTol();

      // update output structure
      ret._batchAccepted = true;
      ret._mi = 0.0;
      ret._rank = _nRank;
      ret._qrTol = _qrTol;
      ret._numIterations = srv.iterations;
      ret._JStart = srv.JStart;
      ret._JFinal = srv.JFinal;
      ret._cholmodMemoryUsage = getCholmodMemoryUsage();
      ret._elapsedTime = Timestamp::now() - timeStart;

      return ret;
    }

    IncrementalEstimator::ReturnValue
        IncrementalEstimator::addBatch(const BatchSP& problem, bool force) {
      // query the time
      const double timeStart = Timestamp::now();

      // insert new batch in the problem
      _problem->add(problem);

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // save design variables in case the batch is rejected
      if (!force)
        _problem->saveDesignVariables();

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // check if the solution is valid
      bool solutionValid = true;
      if (srv.iterations == _optimizer->options().maxIterations ||
          srv.JFinal >= srv.JStart)
        solutionValid = false;

      // return value
      ReturnValue ret;

      // fill the rank and QR tolerance from the linear solver
      ret._rank = _optimizer->getSolver<LinearSolver>()->getRank();
      ret._qrTol = _optimizer->getSolver<LinearSolver>()->getTol();

      // fill statistics from optimizer
      ret._numIterations = srv.iterations;
      ret._JStart = srv.JStart;
      ret._JFinal = srv.JFinal;

      // analyze marginalized system
      const size_t dim = _problem->getGroupDim(_margGroupId);
      const size_t numCols = getJacobianTranspose().rows();
      const double svLogSum = marginalize(getJacobianTranspose(), numCols - dim,
        ret._NS, ret._CS, ret._Sigma, ret._SigmaP, ret._Omega,
        _options._normTol, _options._epsTolSVD);

      // compute MI
      ret._mi = 0.5 * (svLogSum - _svLogSum);

      // batch is kept? MI improvement or rank goes up or force
      bool keepBatch = false;
      if (((ret._mi > _options._miTol || ret._CS.cols() > _CS.cols()) &&
          solutionValid) || force) {
        // warning for rank going down
        if (ret._CS.cols() < _CS.cols() && _options._verbose)
          std::cerr << "WARNING: RANK GOING DOWN!" << std::endl;

        keepBatch = true;

        // update internal variables
        _svLogSum = svLogSum;
        _mi = ret._mi;
        _NS = ret._NS;
        _CS = ret._CS;
        _Sigma = ret._Sigma;
        _SigmaP = ret._SigmaP;
        _Omega = ret._Omega;
        _nRank = ret._rank;
        _qrTol = ret._qrTol;
      }
      ret._batchAccepted = keepBatch;

      // remove batch if necessary
      if (!keepBatch) {
        // restore variables
        _problem->restoreDesignVariables();

        // kick out the problem from the container
        _problem->remove(problem);

        // restore the linear solver
        if (_problem->getNumOptimizationProblems() > 0)
          restoreLinearSolver();
      }

      // insert elapsed time
      ret._elapsedTime = Timestamp::now() - timeStart;

      // insert memory usage
      ret._cholmodMemoryUsage = getCholmodMemoryUsage();

      // output informations
      return ret;
    }

    void IncrementalEstimator::removeBatch(size_t idx) {
      // remove the batch
      _problem->remove(idx);

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // optimize back
      _optimizer->optimize();

      // update mutual information
      const size_t dim = _problem->getGroupDim(_margGroupId);
      const size_t numCols = getJacobianTranspose().rows();
      const double svLogSum = marginalize(getJacobianTranspose(), numCols - dim,
        _NS, _CS, _Sigma, _SigmaP, _Omega, _options._normTol,
        _options._epsTolSVD);
      _mi = svLogSum - _svLogSum; // not really correct! hard to recover.
      _svLogSum = svLogSum;
      _nRank = _optimizer->getSolver<LinearSolver>()->getRank();
      _qrTol = _optimizer->getSolver<LinearSolver>()->getTol();
    }

    void IncrementalEstimator::removeBatch(const BatchSP& batch) {
      auto it = _problem->getOptimizationProblem(batch);
      if (it != _problem->getOptimizationProblemEnd())
        removeBatch(std::distance(_problem->getOptimizationProblemBegin(), it));
    }

    size_t IncrementalEstimator::getNumBatches() const {
      return _problem->getNumOptimizationProblems();
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

    void IncrementalEstimator::restoreLinearSolver() {
      // init the matrix structure
      std::vector<aslam::backend::DesignVariable*> dvs;
      const size_t numDVS = _problem->numDesignVariables();
      dvs.reserve(numDVS);
      size_t columnBase = 0;
      for (size_t i = 0; i < numDVS; ++i) {
        aslam::backend::DesignVariable* dv = _problem->designVariable(i);
        if (dv->isActive()) {
          dvs.push_back(dv);
          dv->setBlockIndex(dvs.size() - 1);
          dv->setColumnBase(columnBase);
          columnBase += dv->minimalDimensions();
        }
      }
      std::vector<aslam::backend::ErrorTerm*> ets;
      const size_t numETS = _problem->numErrorTerms();
      ets.reserve(numETS);
      size_t dim = 0;
      for (size_t i = 0; i < numETS; ++i) {
        aslam::backend::ErrorTerm* et = _problem->errorTerm(i);
        et->setRowBase(dim);
        dim += et->dimension();
        ets.push_back(et);
      }
      _optimizer->getSolver<LinearSolver>()->initMatrixStructure(dvs, ets,
        false);

      // build the system
      _optimizer->getSolver<LinearSolver>()->buildSystem(
        _optimizer->options().nThreads, true);
    }

  }
}
