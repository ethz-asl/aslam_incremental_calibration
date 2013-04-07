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

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"

#include <algorithm>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/algorithms/permute.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalOptimizationProblem::IncrementalOptimizationProblem(
        const DesignVariablesP& designVariables) :
        _designVariablesMarg(designVariables) {
      for (auto it = designVariables.cbegin(); it != designVariables.cend();
          ++it)
        _designVariablesMargLookup.insert(*it);
    }

    IncrementalOptimizationProblem::~IncrementalOptimizationProblem() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t IncrementalOptimizationProblem::getNumOptimizationProblems() const {
      return _optimizationProblems.size();
    }

    const IncrementalOptimizationProblem::OptimizationProblem*
        IncrementalOptimizationProblem::getOptimizationProblem(const
        OptimizationProblemsSPCIt& problemIt) const {
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getOptimizationProblem: "
          "index out of bound", __FILE__, __LINE__);
      const OptimizationProblemSP& problem = _optimizationProblems.at(idx);
      return problem.get();
    }

    IncrementalOptimizationProblem::OptimizationProblem*
        IncrementalOptimizationProblem::getOptimizationProblem(
        const OptimizationProblemsSPIt& problemIt) {
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getOptimizationProblem: "
          "index out of bound", __FILE__, __LINE__);
      OptimizationProblemSP& problem = _optimizationProblems.at(
        std::distance(_optimizationProblems.begin(), problemIt));
      return problem.get();
    }

    const IncrementalOptimizationProblem::OptimizationProblem*
        IncrementalOptimizationProblem::
        getOptimizationProblem(size_t idx) const {
      return getOptimizationProblem(_optimizationProblems.begin() + idx);
    }

    IncrementalOptimizationProblem::OptimizationProblem*
        IncrementalOptimizationProblem::getOptimizationProblem(size_t idx) {
      return getOptimizationProblem(_optimizationProblems.begin() + idx);
    }

    size_t IncrementalOptimizationProblem::
        getNumMarginalizedDesignVariables() const {
      return _designVariablesMarg.size();
    }

    const IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::getMarginalizedDesignVariable(
        const DesignVariablesPCIt dvIt) const {
      const size_t idx =
        std::distance(_designVariablesMarg.begin(), dvIt);
      if (idx >= _designVariablesMarg.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getMarginalizedDesignVariable: "
          "index out of bound", __FILE__, __LINE__);
      return _designVariablesMarg.at(
        std::distance(_designVariablesMarg.begin(), dvIt));
    }

    IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::getMarginalizedDesignVariable(
        const DesignVariablesPIt dvIt) {
      const size_t idx =
        std::distance(_designVariablesMarg.begin(), dvIt);
      if (idx >= _designVariablesMarg.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getMarginalizedDesignVariable: "
          "index out of bound", __FILE__, __LINE__);
      return const_cast<DesignVariable*>(_designVariablesMarg.at(
        std::distance(_designVariablesMarg.begin(), dvIt)));
    }

    const IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::getMarginalizedDesignVariable(
        size_t idx) const {
      return getMarginalizedDesignVariable(_designVariablesMarg.begin() + idx);
    }

    IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::getMarginalizedDesignVariable(size_t
        idx) {
      return getMarginalizedDesignVariable(_designVariablesMarg.begin() + idx);
    }

    const IncrementalOptimizationProblem::DesignVariablesP&
        IncrementalOptimizationProblem::getMarginalizedDesignVariables() const {
      return _designVariablesMarg;
    }

    const IncrementalOptimizationProblem::OptimizationProblemsSP&
        IncrementalOptimizationProblem::getOptimizationProblems() const {
      return _optimizationProblems;
    }

    const IncrementalOptimizationProblem::DesignVariablesP&
        IncrementalOptimizationProblem::getDesignVariables() const {
      return _designVariables;
    }

    const IncrementalOptimizationProblem::ErrorTermsP&
        IncrementalOptimizationProblem::getErrorTerms() const {
      return _errorTerms;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void IncrementalOptimizationProblem::add(
        const OptimizationProblemSP& problem) {

      // update design variable counts
      const size_t numDV = problem->numDesignVariables();
      _designVariablesCounts.reserve(_designVariablesCounts.size() + numDV);
      _designVariables.reserve(_designVariables.size() + numDV);
      for (size_t i = 0; i < numDV; ++i) {
        const aslam::backend::DesignVariable* dv = problem->designVariable(i);
        if (_designVariablesCounts.count(dv) == 0) {
          _designVariablesCounts[dv] = 1;
          if (_designVariablesMargLookup.count(dv) == 0)
            _designVariables.push_back(dv);
        }
        else
          _designVariablesCounts[dv]++;
      }

      // keep trace of the error terms of this problem
      const size_t numET = problem->numErrorTerms();
      _errorTerms.reserve(_errorTerms.size() + numET);
      for (size_t i = 0; i < numET; ++i) {
        const aslam::backend::ErrorTerm* et = problem->errorTerm(i);
        _errorTerms.push_back(et);
      }

      // insert the problem
      _optimizationProblems.push_back(problem);
    }

    void IncrementalOptimizationProblem::remove(
        const OptimizationProblemsSPIt& problemIt) {

      // index bounds check
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::remove: "
          "index out of bound", __FILE__, __LINE__);

      // get the optimization problem to remove
      const OptimizationProblemSP& problem = _optimizationProblems.at(idx);

      // update design variable counts and remove the pointers if necessary
      const size_t numDV = problem->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        const aslam::backend::DesignVariable* dv = problem->designVariable(i);
        _designVariablesCounts[dv]--;
        if (_designVariablesCounts[dv] == 0) {
          _designVariablesCounts.erase(dv);
          auto it = std::find(_designVariables.begin(), _designVariables.end(),
            dv);
          _designVariables.erase(it);
        }
      }

      // remove the error terms pointers of this problem
      // costly if not at the end of the container
      const size_t numET = problem->numErrorTerms();
      if (numET > 0) {
        const aslam::backend::ErrorTerm* et = problem->errorTerm(0);
        auto it = std::find(_errorTerms.begin(), _errorTerms.end(), et);
        _errorTerms.erase(it, it + numET);
      }

      // remove problem from the container
      // costly if not at the end of the container
      _optimizationProblems.erase(problemIt);
    }

    void IncrementalOptimizationProblem::remove(size_t idx) {
      remove(_optimizationProblems.begin() + idx);
    }

    void IncrementalOptimizationProblem::clear() {
      _optimizationProblems.clear();
      _designVariablesCounts.clear();
      _errorTerms.clear();
      _designVariables.clear();
    }

    size_t IncrementalOptimizationProblem::
        numDesignVariablesImplementation() const {
      return _designVariables.size() + _designVariablesMarg.size();
    }

    IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) {
      if (idx >= _designVariables.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::designVariableImplementation: "
          "index out of bound", __FILE__, __LINE__);
      else
        return const_cast<DesignVariable*>(_designVariables[idx]);
    }

    const IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) const {
      if (idx >= numDesignVariables())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::designVariableImplementation: "
          "index out of bound", __FILE__, __LINE__);
      else if (idx >= _designVariables.size())
        return _designVariablesMarg[idx - _designVariablesMarg.size()];
      else
        return _designVariables[idx];
    };

    size_t IncrementalOptimizationProblem::IncrementalOptimizationProblem::
        numErrorTermsImplementation() const {
      return _errorTerms.size();
    }

    IncrementalOptimizationProblem::ErrorTerm*
        IncrementalOptimizationProblem::errorTermImplementation(size_t idx) {
      if (idx >= _errorTerms.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::errorTermImplementation: "
          "index out of bound", __FILE__, __LINE__);
      else
        return const_cast<ErrorTerm*>(_errorTerms[idx]);
    }

    const IncrementalOptimizationProblem::ErrorTerm*
        IncrementalOptimizationProblem::
        errorTermImplementation(size_t idx) const {
      if (idx >= _errorTerms.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::errorTermImplementation: "
          "index out of bound", __FILE__, __LINE__);
      else
        return _errorTerms[idx];
    }

    void IncrementalOptimizationProblem::
        getErrorsImplementation(const DesignVariable* dv,
        std::set<ErrorTerm*>& outErrorSet) {
      throw InvalidOperationException(
        "IncrementalOptimizationProblem::getErrorsImplementation(): "
        "not implemented");
    }


    void IncrementalOptimizationProblem::permuteMarginalizedDesignVariables(
        const std::vector<size_t>& permutation) {
      permute(_designVariablesMarg, permutation);
    }

    void IncrementalOptimizationProblem::permuteDesignVariables(
        const std::vector<size_t>& permutation) {
      permute(_designVariables, permutation);
    }

    void IncrementalOptimizationProblem::permuteErrorTerms(
        const std::vector<size_t>& permutation) {
      permute(_errorTerms, permutation);
    }

  }
}
