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

#include "aslam/calibration/core/OptimizationProblem.h"

#include <utility>

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

    OptimizationProblem::OptimizationProblem() {
    }

    OptimizationProblem::~OptimizationProblem() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const OptimizationProblem::DesignVariableSPGroups&
        OptimizationProblem::getDesignVariablesGroups() const {
      return _designVariables;
    }

    const OptimizationProblem::DesignVariablesSP&
        OptimizationProblem::getDesignVariablesGroup(size_t groupId) const {
      if (isGroupInProblem(groupId))
        return _designVariables.at(groupId);
      else
        throw OutOfBoundException<size_t>(groupId,
          "OptimizationProblem::getDesignVariablesGroup(): "
          "unknown group", __FILE__, __LINE__);
    }

    const OptimizationProblem::ErrorTermsSP&
        OptimizationProblem::getErrorTerms() const {
      return _errorTerms;
    }

    size_t OptimizationProblem::getNumGroups() const {
      return _groupsOrdering.size();
    }

    void OptimizationProblem::
        setGroupsOrdering(const std::vector<size_t>& groupsOrdering) {
      if (groupsOrdering.size() != _groupsOrdering.size())
        throw OutOfBoundException<size_t>(groupsOrdering.size(),
          "OptimizationProblem::setGroupsOrdering(): "
          "wrong groups ordering size", __FILE__, __LINE__);
      std::unordered_set<size_t> groupsLookup;
      for (auto it = groupsOrdering.cbegin(); it != groupsOrdering.cend();
          ++it) {
        if (!isGroupInProblem(*it))
          throw OutOfBoundException<size_t>(*it,
            "OptimizationProblem::setGroupsOrdering(): unknown group",
            __FILE__, __LINE__);
        if (groupsLookup.count(*it))
          throw OutOfBoundException<size_t>(*it,
            "OptimizationProblem::setGroupsOrdering(): duplicate group",
            __FILE__, __LINE__);
        groupsLookup.insert(*it);
      }
      _groupsOrdering = groupsOrdering;
    }

    const std::vector<size_t>& OptimizationProblem::getGroupsOrdering() const {
      return _groupsOrdering;
    }

    size_t OptimizationProblem::
        getGroupId(const DesignVariable* designVariable) const {
      if (isDesignVariableInProblem(designVariable))
        return _designVariablesLookup.at(designVariable);
      else
        throw InvalidOperationException(
          "OptimizationProblem::getGroupId(): "
          "design variable is not in the problem");
    }

    size_t OptimizationProblem::getGroupDim(size_t groupId) const {
      const DesignVariablesSP& designVariables =
        getDesignVariablesGroup(groupId);
      size_t dim = 0;
      for (auto it = designVariables.cbegin(); it != designVariables.cend();
          ++it)
        if ((*it)->isActive())
          dim += (*it)->minimalDimensions();
      return dim;
    }

    bool OptimizationProblem::isGroupInProblem(size_t groupId) const {
      return _designVariables.count(groupId);
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OptimizationProblem::
        addDesignVariable(const DesignVariableSP& designVariable,
        size_t groupId) {
      if (isDesignVariableInProblem(designVariable.get()))
        throw InvalidOperationException(
          "OptimizationProblem::addDesignVariable(): "
          "design variable already included");
      _designVariablesLookup.insert(std::make_pair(designVariable.get(),
        groupId));
      if (!isGroupInProblem(groupId))
        _groupsOrdering.push_back(groupId);
      _designVariables[groupId].push_back(designVariable);
    }

    bool OptimizationProblem::
        isDesignVariableInProblem(const DesignVariable* designVariable) const {
      return _designVariablesLookup.count(designVariable);
    }

    void OptimizationProblem::addErrorTerm(const ErrorTermSP& errorTerm) {
      if (isErrorTermInProblem(errorTerm.get()))
        throw InvalidOperationException(
          "OptimizationProblem::addErrorTerm(): error term already included");
      const size_t numDV = errorTerm->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        const DesignVariable* dv = errorTerm->designVariable(i);
        if (!isDesignVariableInProblem(dv))
          throw InvalidOperationException(
            "OptimizationProblem::addErrorTerm(): "
            "error term contains a design variable not in the problem");
      }
      _errorTermsLookup.insert(errorTerm.get());
      _errorTerms.push_back(errorTerm);
    }

    bool OptimizationProblem::
        isErrorTermInProblem(const ErrorTerm* errorTerm) const {
      return _errorTermsLookup.count(errorTerm);
    }

    void OptimizationProblem::clear() {
      _designVariablesLookup.clear();
      _errorTermsLookup.clear();
      _designVariables.clear();
      _errorTerms.clear();
      _groupsOrdering.clear();
      _designVariablesBackup.clear();
    }

    size_t OptimizationProblem::numDesignVariablesImplementation() const {
      return _designVariablesLookup.size();
    }

    OptimizationProblem::DesignVariable* OptimizationProblem::
        designVariableImplementation(size_t idx) {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return _designVariables.at(groupId)[idxGroup].get();
    }

    const OptimizationProblem::DesignVariable* OptimizationProblem::
        designVariableImplementation(size_t idx) const {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return _designVariables.at(groupId)[idxGroup].get();
    }

    size_t OptimizationProblem::OptimizationProblem::
        numErrorTermsImplementation() const {
      return _errorTermsLookup.size();
    }

    OptimizationProblem::ErrorTerm* OptimizationProblem::
        errorTermImplementation(size_t idx) {
      if (idx >= _errorTerms.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblem::errorTermImplementation(): "
          "index out of bounds", __FILE__, __LINE__);
      else
        return _errorTerms[idx].get();
    }

    const OptimizationProblem::ErrorTerm* OptimizationProblem::
        errorTermImplementation(size_t idx) const {
      if (idx >= _errorTerms.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblem::errorTermImplementation(): "
          "index out of bounds", __FILE__, __LINE__);
      else
        return _errorTerms[idx].get();
    }

    void OptimizationProblem::getErrorsImplementation(const DesignVariable* dv,
        std::set<ErrorTerm*>& outErrorSet) {
      throw InvalidOperationException(
        "OptimizationProblem::getErrorsImplementation(): "
        "not implemented (deprecated)");
    }

    void OptimizationProblem::permuteErrorTerms(const std::vector<size_t>&
        permutation) {
      permute(_errorTerms, permutation);
    }

    void OptimizationProblem::permuteDesignVariables(const std::vector<size_t>&
        permutation, size_t groupId) {
      if (isGroupInProblem(groupId))
        permute(_designVariables.at(groupId), permutation);
      else
        throw OutOfBoundException<size_t>(groupId,
          "OptimizationProblem::permuteDesignVariables(): "
          "unknown group", __FILE__, __LINE__);
    }

    void OptimizationProblem::getGroupId(size_t idx, size_t& groupId,
        size_t& idxGroup) const {
      if (idx >= _designVariablesLookup.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblem::getGroupId(): "
          "index out of bounds", __FILE__, __LINE__);
      size_t idxRunning = 0;
      size_t groupIdRunning = 0;
      for (auto it = _groupsOrdering.cbegin(); it != _groupsOrdering.cend();
          ++it) {
        const DesignVariablesSP& designVariables = _designVariables.at(*it);
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

    void OptimizationProblem::saveDesignVariables() {
      for (auto it = _designVariablesLookup.cbegin();
          it != _designVariablesLookup.cend(); ++it)
        it->first->getParameters(
          _designVariablesBackup[const_cast<DesignVariable*>(it->first)]);
    }

    void OptimizationProblem::restoreDesignVariables() {
      for (auto it = _designVariablesBackup.cbegin();
          it != _designVariablesBackup.cend(); ++it)
        it->first->setParameters(_designVariablesBackup[it->first]);
    }

  }
}
