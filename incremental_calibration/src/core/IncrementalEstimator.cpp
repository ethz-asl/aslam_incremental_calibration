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

#include <aslam/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>

#include "aslam/calibration/core/LinearSolver.h"
#include "aslam/calibration/core/IncrementalOptimizationProblem.h"
#include "aslam/calibration/base/Timestamp.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator(size_t groupId,
        const Options& options, const LinearSolverOptions&
        linearSolverOptions, const OptimizerOptions& optimizerOptions) :
        _options(options),
        _margGroupId(groupId),
        _optimizer(boost::make_shared<Optimizer>(optimizerOptions)),
        _problem(boost::make_shared<IncrementalOptimizationProblem>()),
        _mutualInformation(0),
        _svLog2Sum(0),
        _svdTolerance(0),
        _qrTolerance(-1),
        _svdRank(-1),
        _svdRankDeficiency(-1),
        _qrRank(-1),
        _qrRankDeficiency(-1),
        _peakMemoryUsage(0),
        _memoryUsage(0),
        _numFlops(0) {
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
        _mutualInformation(0),
        _svLog2Sum(0),
        _svdTolerance(0),
        _qrTolerance(-1),
        _svdRank(-1),
        _svdRankDeficiency(-1),
        _qrRank(-1),
        _qrRankDeficiency(-1),
        _peakMemoryUsage(0),
        _memoryUsage(0),
        _numFlops(0) {
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
      _options.miTol = config.getDouble("miTol", _options.miTol);
      _options.verbose = config.getBool("verbose", _options.verbose);
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

    const LinearSolverOptions& IncrementalEstimator::getLinearSolverOptions()
        const {
      return _optimizer->getSolver<LinearSolver>()->getOptions();
    }

    LinearSolverOptions& IncrementalEstimator::getLinearSolverOptions() {
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

    size_t IncrementalEstimator::getMargGroupId() const {
      return _margGroupId;
    }

    double IncrementalEstimator::getMutualInformation() const {
      return _mutualInformation;
    }

    const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
        IncrementalEstimator::getJacobianTranspose() const {
      return _optimizer->getSolver<LinearSolver>()->getJacobianTranspose();
    }

    std::ptrdiff_t IncrementalEstimator::getRank() const {
      return _qrRank;
    }

    std::ptrdiff_t IncrementalEstimator::getRankDeficiency() const {
      return _qrRankDeficiency;
    }

    std::ptrdiff_t IncrementalEstimator::getMarginalRank() const {
      return _svdRank;
    }

    std::ptrdiff_t IncrementalEstimator::getMarginalRankDeficiency() const {
      return _svdRankDeficiency;
    }

    double IncrementalEstimator::getSVDTolerance() const {
      return _svdTolerance;
    }

    double IncrementalEstimator::getQRTolerance() const {
      return _qrTolerance;
    }

    size_t IncrementalEstimator::getPeakMemoryUsage() const {
      return _peakMemoryUsage;
    }

    size_t IncrementalEstimator::getMemoryUsage() const {
      return _memoryUsage;
    }

    double IncrementalEstimator::getNumFlops() const {
      return _numFlops;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedNullSpace()
        const {
      return _nullSpace;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedColumnSpace()
        const {
      return _columnSpace;
    }

    const Eigen::MatrixXd& IncrementalEstimator::getMarginalizedCovariance()
        const {
      return _covariance;
    }

    const Eigen::MatrixXd&
        IncrementalEstimator::getProjectedMarginalizedCovariance() const {
      return _projectedCovariance;
    }

    const Eigen::VectorXd& IncrementalEstimator::getSingularValues() const {
      return _singularValues;
    }

    const Eigen::VectorXd& IncrementalEstimator::getScaledSingularValues()
        const {
      return _scaledSingularValues;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    IncrementalEstimator::ReturnValue IncrementalEstimator::reoptimize() {
      // query the time
      const double timeStart = Timestamp::now();

      // ensure marginalized design variables are well located
      orderMarginalizedDesignVariables();

      // set the marginalization index of the linear solver
      size_t JCols = 0;
      for (auto it = _problem->getGroupsOrdering().cbegin();
          it != _problem->getGroupsOrdering().cend(); ++it)
        JCols += _problem->getGroupDim(*it);
      const size_t dim = _problem->getGroupDim(_margGroupId);
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->setMargStartIndex(JCols - dim);

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // grep the scaled singular values if scaling enabled
      if (linearSolver->getOptions().columnScaling)
        _scaledSingularValues = linearSolver->getSingularValues();
      else
        _scaledSingularValues.resize(0);

      // analyze marginal system
      linearSolver->analyzeMarginal();

      // retrieve informations from the linear solver
      _mutualInformation = 0;
      _svLog2Sum = linearSolver->getSingularValuesLog2Sum();
      _nullSpace = linearSolver->getNullSpace();
      _columnSpace = linearSolver->getColumnSpace();
      _covariance = linearSolver->getCovariance();
      _projectedCovariance = linearSolver->getProjectedCovariance();
      _singularValues = linearSolver->getSingularValues();
      _svdTolerance = linearSolver->getSVDTolerance();
      _qrTolerance = linearSolver->getQRTolerance();
      _svdRank = linearSolver->getSVDRank();
      _svdRankDeficiency = linearSolver->getSVDRankDeficiency();
      _qrRank = linearSolver->getQRRank();
      _qrRankDeficiency = linearSolver->getQRRankDeficiency();
      _peakMemoryUsage = linearSolver->getPeakMemoryUsage();
      _memoryUsage = linearSolver->getMemoryUsage();
      _numFlops = linearSolver->getNumFlops();

      // update output structure
      ReturnValue ret;
      ret.batchAccepted = true;
      ret.mutualInformation = 0.0;
      ret.rank = _qrRank;
      ret.rankDeficiency = _qrRankDeficiency;
      ret.marginalRank = _svdRank;
      ret.marginalRankDeficiency = _svdRankDeficiency;
      ret.svdTolerance = _svdTolerance;
      ret.qrTolerance = _qrTolerance;
      ret.nullSpace = _nullSpace;
      ret.columnSpace = _columnSpace;
      ret.covariance = _covariance;
      ret.projectedCovariance = _projectedCovariance;
      ret.singularValues = _singularValues;
      ret.scaledSingularValues = _scaledSingularValues;
      ret.numIterations = srv.iterations;
      ret.JStart = srv.JStart;
      ret.JFinal = srv.JFinal;
      ret.elapsedTime = Timestamp::now() - timeStart;
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

      // set the marginalization index of the linear solver
      size_t JCols = 0;
      for (auto it = _problem->getGroupsOrdering().cbegin();
          it != _problem->getGroupsOrdering().cend(); ++it)
        JCols += _problem->getGroupDim(*it);
      const size_t dim = _problem->getGroupDim(_margGroupId);
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->setMargStartIndex(JCols - dim);

      // optimize
      aslam::backend::SolutionReturnValue srv = _optimizer->optimize();

      // return value
      ReturnValue ret;

      // fill statistics from optimizer
      ret.numIterations = srv.iterations;
      ret.JStart = srv.JStart;
      ret.JFinal = srv.JFinal;

      // grep the scaled singular values if scaling enabled
      if (linearSolver->getOptions().columnScaling)
        ret.scaledSingularValues = linearSolver->getSingularValues();
      else
        ret.scaledSingularValues.resize(0);

      // analyze marginal system
      linearSolver->analyzeMarginal();

      // fill statistics from the linear solver
      ret.rank = linearSolver->getQRRank();
      ret.rankDeficiency = linearSolver->getQRRankDeficiency();
      ret.marginalRank = linearSolver->getSVDRank();
      ret.marginalRankDeficiency = linearSolver->getSVDRankDeficiency();
      ret.svdTolerance = linearSolver->getSVDTolerance();
      ret.qrTolerance = linearSolver->getQRTolerance();
      ret.nullSpace = linearSolver->getNullSpace();
      ret.columnSpace = linearSolver->getColumnSpace();
      ret.covariance = linearSolver->getCovariance();
      ret.projectedCovariance = linearSolver->getProjectedCovariance();
      ret.singularValues = linearSolver->getSingularValues();

      // check if the solution is valid
      bool solutionValid = true;
      if (srv.iterations == _optimizer->options().maxIterations ||
          srv.JFinal >= srv.JStart)
        solutionValid = false;

      // compute the mutual information
      const double svLog2Sum = linearSolver->getSingularValuesLog2Sum();
      ret.mutualInformation = 0.5 * (svLog2Sum - _svLog2Sum);

      // batch is kept? MI improvement or rank goes up or force
      bool keepBatch = false;
      if (((ret.mutualInformation > _options.miTol ||
          ret.marginalRank > _svdRank) && solutionValid) || force) {
        // warning for rank going down
        if (ret.marginalRank < _svdRank && _options.verbose)
          std::cerr << "IncrementalEstimator::addBatch(): "
            "WARNING: RANK GOING DOWN!" << std::endl;

        keepBatch = true;

        // update internal variables
        _mutualInformation = ret.mutualInformation;
        _svLog2Sum = svLog2Sum;
        _nullSpace = ret.nullSpace;
        _columnSpace = ret.columnSpace;
        _covariance = ret.covariance;
        _projectedCovariance = ret.projectedCovariance;
        _singularValues = ret.singularValues;
        _scaledSingularValues = ret.scaledSingularValues;
        _svdTolerance = ret.svdTolerance;
        _qrTolerance = ret.qrTolerance;
        _svdRank = ret.marginalRank;
        _svdRankDeficiency = ret.marginalRankDeficiency;
        _qrRank = ret.rank;
        _qrRankDeficiency = ret.rankDeficiency;
        _peakMemoryUsage = linearSolver->getPeakMemoryUsage();
        _memoryUsage = linearSolver->getMemoryUsage();
        _numFlops = linearSolver->getNumFlops();
      }
      ret.batchAccepted = keepBatch;

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
      ret.elapsedTime = Timestamp::now() - timeStart;

      // output informations
      return ret;
    }

    void IncrementalEstimator::removeBatch(size_t idx) {
      // remove the batch
      _problem->remove(idx);

      // reoptimize
      reoptimize();
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
      auto linearSolver = _optimizer->getSolver<LinearSolver>();
      linearSolver->initMatrixStructure(dvs, ets, false);

      // build the system
      linearSolver->buildSystem(_optimizer->options().nThreads, true);
    }

  }
}
