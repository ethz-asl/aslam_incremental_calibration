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

/** \file IncrementalOptimizationProblem.h
    \brief This file defines the IncrementalOptimizationProblem class, which is
           a container for optimization problems.
  */

#ifndef ASLAM_CALIBRATION_CORE_INCREMENTAL_OPTIMIZATION_PROBLEM_H
#define ASLAM_CALIBRATION_CORE_INCREMENTAL_OPTIMIZATION_PROBLEM_H

#include <set>
#include <vector>
#include <unordered_map>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/OptimizationProblemBase.hpp>

namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;

  }
  namespace calibration {

    class OptimizationProblem;

    /** The class IncrementalOptimizationProblem implements a container for
        optimization problems.
        TODO: * error terms lookup? for isErrorTermInProblem()
              * optimization problems lookup?
        \brief Incremental optimization problem
      */
    class IncrementalOptimizationProblem :
      public aslam::backend::OptimizationProblemBase {
    public:
      /** \name Types definitions
        @{
        */
      /// Optimization problem type (shared_ptr)
      typedef boost::shared_ptr<OptimizationProblem> OptimizationProblemSP;
      /// Optimization problem container (shared_ptr)
      typedef std::vector<OptimizationProblemSP> OptimizationProblemsSP;
      /// Optimization problem container iteraror (shared_ptr)
      typedef OptimizationProblemsSP::iterator OptimizationProblemsSPIt;
      /// Optimization problem container constant iteraror (shared_ptr)
      typedef OptimizationProblemsSP::const_iterator OptimizationProblemsSPCIt;
      /// Design variable type
      typedef aslam::backend::DesignVariable DesignVariable;
      /// Design variable (pointer) to count, group ID container
      typedef std::unordered_map<const DesignVariable*,
        std::pair<size_t, size_t> > DesignVariablesPCountId;
      /// Container for design variables (pointer)
      typedef std::vector<const DesignVariable*> DesignVariablesP;
      /// Container for design variable groups
      typedef std::unordered_map<size_t, DesignVariablesP>
        DesignVariablePGroups;
      /// Error term type
      typedef aslam::backend::ErrorTerm ErrorTerm;
      /// Error term container (pointer)
      typedef std::vector<const ErrorTerm*> ErrorTermsP;
      /// Self type
      typedef IncrementalOptimizationProblem Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      IncrementalOptimizationProblem();
      /// Copy constructor
      IncrementalOptimizationProblem(const Self& other) = delete;
      /// Copy assignment operator
      IncrementalOptimizationProblem& operator = (const Self& other) = delete;
      /// Move constructor
      IncrementalOptimizationProblem(Self&& other) = delete;
      /// Move assignment operator
      IncrementalOptimizationProblem& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~IncrementalOptimizationProblem();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Inserts an optimization problem
      void add(const OptimizationProblemSP& problem);
      /// Remove an optimization problem
      void remove(const OptimizationProblemsSPIt& problemIt);
      /// Remove an optimization problem
      void remove(size_t idx);
      /// Permutes the design variables in a group
      void permuteDesignVariables(const std::vector<size_t>& permutation,
        size_t groupId);
      /// Permutes the error terms
      void permuteErrorTerms(const std::vector<size_t>& permutation);
      /// Clears the content of the problem
      void clear();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the number of stored optimization problems
      size_t getNumOptimizationProblems() const;
      /// Returns an optimization problem from an iterator
      const OptimizationProblem* getOptimizationProblem(const
        OptimizationProblemsSPCIt& problemIt) const;
      /// Returns an optimization problem from an iterator
      OptimizationProblem* getOptimizationProblem(const
        OptimizationProblemsSPIt& problemIt);
      /// Returns an optimization problem from an index
      const OptimizationProblem* getOptimizationProblem(size_t idx) const;
      /// Returns an optimization problem from an index
      OptimizationProblem* getOptimizationProblem(size_t idx);
      /// Returns the optimization problems
      const OptimizationProblemsSP& getOptimizationProblems() const;
      /// Checks if a design variable is in the problem
      bool isDesignVariableInProblem(const DesignVariable* designVariable)
        const;
      /// Checks if an error term is in the problem
      bool isErrorTermInProblem(const ErrorTerm* errorTerm) const;
      /// Returns the design variables groups
      const DesignVariablePGroups& getDesignVariablesGroups() const;
      /// Returns the design variables associated with a group
      const DesignVariablesP& getDesignVariablesGroup(size_t groupId) const;
      /// Returns the error terms
      const ErrorTermsP& getErrorTerms() const;
      /// Returns the number of groups
      size_t getNumGroups() const;
      /// Sets the groups ordering
      void setGroupsOrdering(const std::vector<size_t>& groupsOrdering);
      /// Returns the groups ordering
      const std::vector<size_t>& getGroupsOrdering() const;
      /// Returns the group id of a design variable
      size_t getGroupId(const DesignVariable* designVariable) const;
      /// Returns the dimension of a group
      size_t getGroupDim(size_t groupId) const;
      /// Checks if a group is in the problem
      bool isGroupInProblem(size_t groupId) const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Returns the number of design variables in the problem
      virtual size_t numDesignVariablesImplementation() const;
      /// Returns design variable indexed by idx
      virtual DesignVariable* designVariableImplementation(size_t idx);
      /// Returns design variable indexed by idx
      virtual const DesignVariable* designVariableImplementation(size_t idx)
        const;
      /// Returns the number of error terms in the problem
      virtual size_t numErrorTermsImplementation() const;
      /// Returns error term indexed by idx
      virtual ErrorTerm* errorTermImplementation(size_t idx);
      /// Returns error term indexed by idx
      virtual const ErrorTerm* errorTermImplementation(size_t idx) const;
      /// Returns error terms associated with a design variable
      virtual void getErrorsImplementation(const DesignVariable* dv,
        std::set<ErrorTerm*>& outErrorSet);
      /// Returns the group id an index falls in
      void getGroupId(size_t idx, size_t& groupId, size_t& idxGroup) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Optimization problems shared pointers
      OptimizationProblemsSP _optimizationProblems;
      /// Design variable pointers counts and group ID
      DesignVariablesPCountId _designVariablesCounts;
      /// Storage for the design variables pointers in groups
      DesignVariablePGroups _designVariables;
      /// Groups ordering
      std::vector<size_t> _groupsOrdering;
      /// Error terms pointers
      ErrorTermsP _errorTerms;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_OPTIMIZATION_PROBLEM_H
