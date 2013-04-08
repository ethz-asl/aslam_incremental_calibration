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

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

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

    const OptimizationProblem::DesignVariablesSP&
        OptimizationProblem::getDesignVariables() const {
      return _designVariables;
    }

    const OptimizationProblem::ErrorTermsSP&
        OptimizationProblem::getErrorTerms() const {
      return _errorTerms;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void OptimizationProblem::
        addDesignVariable(const DesignVariableSP& designVariable) {
      if (isDesignVariableInProblem(designVariable.get()))
        throw InvalidOperationException(
          "OptimizationProblem::addDesignVariable(): "
          "design variable already included");
      _designVariablesLookup.insert(designVariable.get());
      _designVariables.push_back(designVariable);
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
    }

    size_t OptimizationProblem::numDesignVariablesImplementation() const {
      return _designVariablesLookup.size();
    }

    OptimizationProblem::DesignVariable* OptimizationProblem::
        designVariableImplementation(size_t idx) {
      if (idx >= _designVariables.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblem::designVariableImplementation(): "
          "index out of bounds", __FILE__, __LINE__);
      else
        return _designVariables[idx].get();
    }

    const OptimizationProblem::DesignVariable* OptimizationProblem::
        designVariableImplementation(size_t idx) const {
      if (idx >= _designVariables.size())
        throw OutOfBoundException<size_t>(idx,
          "OptimizationProblem::designVariableImplementation(): "
          "index out of bounds", __FILE__, __LINE__);
      else
        return _designVariables[idx].get();
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
        "OptimizationProblem::getErrorsImplementation(): not implemented");
    }

  }
}
