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

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/Optimizer2.hpp>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Static Members Initialization                                              */
/******************************************************************************/

const struct IncrementalEstimator::Options
  IncrementalEstimator::_defaultOptions = {0.5, 0.02};

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalEstimator::IncrementalEstimator(const DVContainer&
        designVariablesMarg, const DVContainer& designVariablesInv,
        const Options& options) :
        _designVariablesMarg(designVariablesMarg),
        _designVariablesInv(designVariablesInv),
        _sumLogDiagROld(0),
        _options(options) {
    }

    IncrementalEstimator::~IncrementalEstimator() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const IncrementalEstimator::ETContainer&
        IncrementalEstimator::getErrorTermsInfo() const {
      return _errorTermsInfo;
    }

    IncrementalEstimator::ETContainer&
        IncrementalEstimator::getErrorTermsInfo() {
      return _errorTermsInfo;
    }

    IncrementalEstimator::ETContainerIt IncrementalEstimator::getETBegin() {
      return _errorTermsInfo.begin();
    }

    IncrementalEstimator::ETContainerConstIt
        IncrementalEstimator::getETCBegin() const {
      return _errorTermsInfo.cbegin();
    }

    IncrementalEstimator::ETContainerIt IncrementalEstimator::getETEnd() {
      return _errorTermsInfo.end();
    }

    IncrementalEstimator::ETContainerConstIt
        IncrementalEstimator::getETCEnd() const {
      return _errorTermsInfo.cend();
    }

    const IncrementalEstimator::DVContainer&
        IncrementalEstimator::getDesignVariablesMarg() const {
      return _designVariablesMarg;
    }

    IncrementalEstimator::DVContainer&
        IncrementalEstimator::getDesignVariablesMarg() {
      return _designVariablesMarg;
    }

    IncrementalEstimator::DVContainerIt IncrementalEstimator::getDVMBegin() {
      return _designVariablesMarg.begin();
    }

    IncrementalEstimator::DVContainerConstIt
        IncrementalEstimator::getDVMCBegin() const {
      return _designVariablesMarg.cbegin();
    }

    IncrementalEstimator::DVContainerIt IncrementalEstimator::getDVMEnd() {
      return _designVariablesMarg.end();
    }

    IncrementalEstimator::DVContainerConstIt
        IncrementalEstimator::getDVMCEnd() const {
      return _designVariablesMarg.cend();
    }

    const IncrementalEstimator::DVContainer&
        IncrementalEstimator::getDesignVariablesInfo() const {
      return _designVariablesInfo;
    }

    IncrementalEstimator::DVContainer&
        IncrementalEstimator::getDesignVariablesInfo() {
      return _designVariablesInfo;
    }

    IncrementalEstimator::DVContainerIt IncrementalEstimator::getDVIBegin() {
      return _designVariablesInfo.begin();
    }

    IncrementalEstimator::DVContainerConstIt
        IncrementalEstimator::getDVICBegin() const {
      return _designVariablesInfo.cbegin();
    }

    IncrementalEstimator::DVContainerIt IncrementalEstimator::getDVIEnd() {
      return _designVariablesInfo.end();
    }

    IncrementalEstimator::DVContainerConstIt
        IncrementalEstimator::getDVICEnd() const {
      return _designVariablesInfo.cend();
    }

    const IncrementalEstimator::Options&
        IncrementalEstimator::getOptions() const {
      return _options;
    }

    IncrementalEstimator::Options& IncrementalEstimator::getOptions() {
      return _options;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    bool IncrementalEstimator::addMeasurementBatch(const ETContainer&
        errorTermsNew, const DVContainer& designVariablesNew) {

      // create optimization problem
      boost::shared_ptr<aslam::backend::OptimizationProblem> problem(
        new aslam::backend::OptimizationProblem);

      // add design variables from the informative set to the problem
      for (auto it = _designVariablesInfo.cbegin();
          it != _designVariablesInfo.cend(); ++it)
        problem->addDesignVariable(*it);

      // add design variables from the new set to the problem
      for (auto it = designVariablesNew.cbegin();
          it != designVariablesNew.cend(); ++it)
        problem->addDesignVariable(*it);

      // add design variables from the time-invariant set to the problem
      for (auto it = _designVariablesInv.cbegin();
          it != _designVariablesInv.cend(); ++it)
        problem->addDesignVariable(*it);

      // add design variables from the marginalized set to the problem
      size_t dim = 0;
      for (auto it = _designVariablesMarg.cbegin();
          it != _designVariablesMarg.cend(); ++it) {
        problem->addDesignVariable(*it);
        dim += (*it)->minimalDimensions();
      }

      // add error terms from the informative set
      for (auto it = _errorTermsInfo.cbegin();
          it != _errorTermsInfo.cend(); ++it)
        problem->addErrorTerm(*it);

      // add error terms from the new set
      for (auto it = errorTermsNew.cbegin(); it != errorTermsNew.cend(); ++it)
        problem->addErrorTerm(*it);

      // optimization options
      aslam::backend::Optimizer2Options options;
      options.verbose = true;
      options.doLevenbergMarquardt = false;
      options.linearSolver = "sparse_qr";

      // linear solver options
      aslam::backend::SparseQRLinearSolverOptions linearSolverOptions;
      linearSolverOptions.colNorm = true;
      linearSolverOptions.qrTol = _options._qrTol;

      // create optimizer with given options
      aslam::backend::Optimizer2 optimizer(options);

      // set options to the linear solver
      optimizer.getSolver<LinearSolver>()->setOptions(linearSolverOptions);

      // set the constructed problem to the optimizer and optimize
      optimizer.setProblem(problem);
      optimizer.optimize();

      // compute sum of log of the diagonal elements of R
      const double sumLogDiagR = optimizer.getSolver<LinearSolver>()
        ->computeSumLogDiagR(3);

      // batch is kept?
      bool keepBatch = false;

      // first round of estimation?
      if (!_sumLogDiagROld) {
        _sumLogDiagROld = sumLogDiagR;
        keepBatch = true;
      }
      else {
        // compute MI
        const double mi = sumLogDiagR - _sumLogDiagROld;

        // MI improvement
        if (mi > _options._miTol) {
          _sumLogDiagROld = sumLogDiagR;
          keepBatch = true;
        }
      }

      // keep batch if necessary
      if (keepBatch) {
        // add the new error terms into the informative set
        _errorTermsInfo.reserve(_errorTermsInfo.size() + errorTermsNew.size());
        _errorTermsInfo.insert(_errorTermsInfo.end(), errorTermsNew.begin(),
          errorTermsNew.end());

        // add the new design variables into the informative set
        _designVariablesInfo.reserve(_designVariablesInfo.size() +
          designVariablesNew.size());
        _designVariablesInfo.insert(_designVariablesInfo.end(),
          designVariablesNew.begin(), designVariablesNew.end());
      }

      return keepBatch;
    }

  }
}
