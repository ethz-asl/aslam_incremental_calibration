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
#include <unordered_set>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/algorithms/permute.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalOptimizationProblem::IncrementalOptimizationProblem() {
    }

    IncrementalOptimizationProblem::~IncrementalOptimizationProblem() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t IncrementalOptimizationProblem::getNumOptimizationProblems() const {
      return _optimizationProblems.size();
    }

    const OptimizationProblem*
        IncrementalOptimizationProblem::getOptimizationProblem(
        const OptimizationProblemsSPCIt& problemIt) const {
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getOptimizationProblem: "
          "index out of bound", __FILE__, __LINE__);
      const OptimizationProblemSP& problem = _optimizationProblems.at(idx);
      return problem.get();
    }

    OptimizationProblem* IncrementalOptimizationProblem::getOptimizationProblem(
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

    const OptimizationProblem* IncrementalOptimizationProblem::
        getOptimizationProblem(size_t idx) const {
      return getOptimizationProblem(_optimizationProblems.begin() + idx);
    }

    OptimizationProblem* IncrementalOptimizationProblem::
        getOptimizationProblem(size_t idx) {
      return getOptimizationProblem(_optimizationProblems.begin() + idx);
    }

    const IncrementalOptimizationProblem::OptimizationProblemsSP&
        IncrementalOptimizationProblem::getOptimizationProblems() const {
      return _optimizationProblems;
    }

    bool IncrementalOptimizationProblem::
        isDesignVariableInProblem(const DesignVariable* designVariable) const {
      return _designVariablesCounts.count(designVariable);
    }

    bool IncrementalOptimizationProblem::isErrorTermInProblem(const ErrorTerm*
        errorTerm) const {
      // TODO: implement lookup structure
      return false;
    }

    const IncrementalOptimizationProblem::DesignVariablePGroups&
        IncrementalOptimizationProblem::getDesignVariablesGroups() const {
      return _designVariables;
    }

    const IncrementalOptimizationProblem::DesignVariablesP&
        IncrementalOptimizationProblem::
        getDesignVariablesGroup(size_t groupId) const {
      if (isGroupInProblem(groupId))
        return _designVariables.at(groupId);
      else
        throw OutOfBoundException<size_t>(groupId,
          "IncrementalOptimizationProblem::getDesignVariablesGroup(): "
          "unknown group", __FILE__, __LINE__);
    }

    const IncrementalOptimizationProblem::ErrorTermsP&
        IncrementalOptimizationProblem::getErrorTerms() const {
      return _errorTerms;
    }

    size_t IncrementalOptimizationProblem::getNumGroups() const {
      return _designVariables.size();
    }

    void IncrementalOptimizationProblem::
        setGroupsOrdering(const std::vector<size_t>& groupsOrdering) {
      if (groupsOrdering.size() != _groupsOrdering.size())
        throw OutOfBoundException<size_t>(groupsOrdering.size(),
          "IncrementalOptimizationProblem::setGroupsOrdering(): "
          "wrong groups ordering size", __FILE__, __LINE__);
      std::unordered_set<size_t> groupsLookup;
      for (auto it = groupsOrdering.cbegin(); it != groupsOrdering.cend();
          ++it) {
        if (!isGroupInProblem(*it))
          throw OutOfBoundException<size_t>(*it,
            "IncrementalOptimizationProblem::setGroupsOrdering(): "
            "unknown group",
            __FILE__, __LINE__);
        if (groupsLookup.count(*it))
          throw OutOfBoundException<size_t>(*it,
            "IncrementalOptimizationProblem::setGroupsOrdering(): "
            "duplicate group",
            __FILE__, __LINE__);
        groupsLookup.insert(*it);
      }
      _groupsOrdering = groupsOrdering;
    }

    const std::vector<size_t>&
        IncrementalOptimizationProblem::getGroupsOrdering() const {
      return _groupsOrdering;
    }

    size_t IncrementalOptimizationProblem::
        getGroupId(const DesignVariable* designVariable) const {
      if (isDesignVariableInProblem(designVariable))
        return _designVariablesCounts.at(designVariable).second;
      else
        throw InvalidOperationException(
          "IncrementalOptimizationProblem::getGroupId(): "
          "design variable is not in the problem");
    }

    size_t IncrementalOptimizationProblem::getGroupDim(size_t groupId) const {
      const DesignVariablesP& designVariables =
        getDesignVariablesGroup(groupId);
      size_t dim = 0;
      for (auto it = designVariables.cbegin(); it != designVariables.cend();
          ++it)
        dim += (*it)->minimalDimensions();
      return dim;
    }

    bool IncrementalOptimizationProblem::
        isGroupInProblem(size_t groupId) const {
      return _designVariables.count(groupId);
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void IncrementalOptimizationProblem::add(
        const OptimizationProblemSP& problem) {

      // update design variable counts, grouping, and storing
      const size_t numDV = problem->numDesignVariables();
      _designVariablesCounts.reserve(_designVariablesCounts.size() + numDV);
      for (size_t i = 0; i < numDV; ++i) {
        const DesignVariable* dv = problem->designVariable(i);
        const size_t groupId = problem->getGroupId(dv);
        if (!isGroupInProblem(groupId))
          _groupsOrdering.push_back(groupId);
        if (!isDesignVariableInProblem(dv)) {
          _designVariablesCounts.insert(std::make_pair(dv,
            std::make_pair(1, groupId)));
          _designVariables[groupId].push_back(dv);
        }
        else {
          if (getGroupId(dv) != groupId)
            throw InvalidOperationException(
              "IncrementalOptimizationProblem::add(): group mismatch");
          _designVariablesCounts[dv].first++;
        }
      }

      // keep trace of the error terms of this problem
      const size_t numET = problem->numErrorTerms();
      _errorTerms.reserve(_errorTerms.size() + numET);
      for (size_t i = 0; i < numET; ++i) {
        const ErrorTerm* et = problem->errorTerm(i);
        if (isErrorTermInProblem(et))
          throw InvalidOperationException(
            "IncrementalOptimizationProblem::add(): "
            "error term already in the problem");
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
        const DesignVariable* dv = problem->designVariable(i);
        _designVariablesCounts[dv].first--;
        if (_designVariablesCounts[dv].first == 0) {
          const size_t groupId = getGroupId(dv);
          _designVariablesCounts.erase(dv);
          auto it = std::find(_designVariables[groupId].begin(),
            _designVariables[groupId].end(), dv);
          _designVariables[groupId].erase(it);
          if (_designVariables[groupId].empty())
            _designVariables.erase(groupId);
        }
      }

      // remove the error terms pointers of this problem
      // costly if not at the end of the container
      const size_t numET = problem->numErrorTerms();
      if (numET > 0) {
        const ErrorTerm* et = problem->errorTerm(0);
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
      _designVariables.clear();
      _groupsOrdering.clear();
      _errorTerms.clear();
    }

    size_t IncrementalOptimizationProblem::
        numDesignVariablesImplementation() const {
      return _designVariablesCounts.size();
    }

    IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return const_cast<DesignVariable*>(
        _designVariables.at(groupId)[idxGroup]);
    }

    const IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) const {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return _designVariables.at(groupId)[idxGroup];
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
        "not implemented (deprecated)");
    }

    void IncrementalOptimizationProblem::
        permuteErrorTerms(const std::vector<size_t>& permutation) {
      permute(_errorTerms, permutation);
    }

    void IncrementalOptimizationProblem::permuteDesignVariables(
        const std::vector<size_t>& permutation, size_t groupId) {
      if (isGroupInProblem(groupId))
        permute(_designVariables.at(groupId), permutation);
      else
        throw OutOfBoundException<size_t>(groupId,
          "IncrementalOptimizationProblem::permuteDesignVariables(): "
          "unknown group", __FILE__, __LINE__);
    }

    void IncrementalOptimizationProblem::getGroupId(size_t idx, size_t& groupId,
        size_t& idxGroup) const {
      if (idx >= _designVariablesCounts.size())
        throw OutOfBoundException<size_t>(idx,
          "IncrementalOptimizationProblem::getGroupId(): "
          "index out of bounds", __FILE__, __LINE__);
      size_t idxRunning = 0;
      size_t groupIdRunning = 0;
      for (auto it = _groupsOrdering.cbegin(); it != _groupsOrdering.cend();
          ++it) {
        const DesignVariablesP& designVariables = _designVariables.at(*it);
        const size_t groupSize = designVariables.size();
        if ((idxRunning + groupSize) > idx) {
          groupIdRunning = *it;
          break;
        }
        else
          idxRunning += groupSize;
      }
      groupId = groupIdRunning;
      idxGroup = idx - idxRunning;
    }

  }
}
