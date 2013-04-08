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
#include <unordered_set>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/OptimizationProblemBase.hpp>

namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;

  }
  namespace calibration {

    /** The class IncrementalOptimizationProblem implements a container for
        optimization problems.
        \brief Incremental optimization problem
      */
    class IncrementalOptimizationProblem :
      public aslam::backend::OptimizationProblemBase {
    public:
      /** \name Types definitions
        @{
        */
      /// Optimization problem type
      typedef aslam::backend::OptimizationProblemBase OptimizationProblem;
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
      /// Design variable (pointer) to count container
      typedef std::unordered_map<const DesignVariable*, size_t>
        DesignVariablesPCount;
      /// Design variable (pointer) lookup
      typedef std::unordered_set<const DesignVariable*> DesignVariablesPLookup;
      /// Design variable container (pointer)
      typedef std::vector<const DesignVariable*> DesignVariablesP;
      /// Design variable container (pointer) iterator
      typedef DesignVariablesP::iterator DesignVariablesPIt;
      /// Design variable container (pointer) constant iterator
      typedef DesignVariablesP::const_iterator DesignVariablesPCIt;
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
      /// Constructor with the marginalized variables
      IncrementalOptimizationProblem(const DesignVariablesP& designVariables);
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
      /// Clears the content of the problem
      void clear();
      /// Apply a permutation to the marginalized design variables
      void permuteMarginalizedDesignVariables(const std::vector<size_t>&
        permutation);
      /// Apply a permutation to the design variables
      void permuteDesignVariables(const std::vector<size_t>& permutation);
      /// Apply a permutation to the error terms
      void permuteErrorTerms(const std::vector<size_t>& permutation);
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
      /// Returns the number of marginalized design variables
      size_t getNumMarginalizedDesignVariables() const;
      /// Returns a marginalized design variable from an iterator
      const DesignVariable* getMarginalizedDesignVariable(
        const DesignVariablesPCIt dvIt) const;
      /// Returns a marginalized design variable from an iterator
      DesignVariable* getMarginalizedDesignVariable(const DesignVariablesPIt
        dvIt);
      /// Returns a marginalized design variable from an index
      const DesignVariable* getMarginalizedDesignVariable(size_t idx) const;
      /// Returns a marginalized design variable from an index
      DesignVariable* getMarginalizedDesignVariable(size_t idx);
      /// Returns the marginalized design variables
      const DesignVariablesP& getMarginalizedDesignVariables() const;
      /// Returns the dimension of the marginalized design variables
      size_t getMarginalizedDesignVariablesDim() const;
      /// Returns the design variables
      const DesignVariablesP& getDesignVariables() const;
      /// Returns the error terms
      const ErrorTermsP& getErrorTerms() const;
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
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Optimization problems shared pointers
      OptimizationProblemsSP _optimizationProblems;
      /// Design variable pointers counts
      DesignVariablesPCount _designVariablesCounts;
      /// Marginalized design variables pointers
      DesignVariablesP _designVariablesMarg;
      /// Fast lookup of marginalized design variables pointers
      DesignVariablesPLookup _designVariablesMargLookup;
      /// Marginalized design variables dimension
      size_t _designVariablesMargDim;
      /// Design variables pointers
      DesignVariablesP _designVariables;
      /// Error terms pointers
      ErrorTermsP _errorTerms;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_OPTIMIZATION_PROBLEM_H
