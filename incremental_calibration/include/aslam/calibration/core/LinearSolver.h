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

/** \file LinearSolver.h
    \brief This file defines the LinearSolver class, which is a specific linear
           solver for incremental calibration problems.
  */

#ifndef ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H
#define ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H

#include <cstddef>

#include <vector>
#include <string>
#include <limits>

#include <cholmod.h>

#include <Eigen/Core>

#include <aslam/backend/LinearSystemSolver.hpp>

template <typename Entry> struct SuiteSparseQR_factorization;

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm; 

  }
  namespace calibration {

    /** The class LinearSolver implements a specific linear solver for
        incremental calibration problems. It uses a combination of SPQR and SVD.
        The right part of the input matrix can be marginalized out and solved
        by SVD, while the rest of the variables are solved with SPQR.
        \brief Linear solver for incremental calibration
      */
    class LinearSolver :
      public aslam::backend::LinearSystemSolver {
    public:
      /** \name Types definitions
        @{
        */
      /// Options for the linear solver
      struct Options {
        /// Default constructor
        Options() :
            columnScaling(false),
            epsNorm(std::numeric_limits<double>::epsilon()),
            epsRank(std::numeric_limits<double>::epsilon()),
            epsQR(std::numeric_limits<double>::epsilon()) {}
        /// Perform column scaling/normalization
        bool columnScaling;
        /// Epsilon for when to consider an element being zero in the norm
        double epsNorm;
        /// Epsilon for numerical rank tolerance
        double epsRank;
        /// Epsilon for QR tolerance computation
        double epsQR;
      };
      /// Self type
      typedef LinearSolver Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor with options structure
      LinearSolver(const Options& options = Options());
      /// Constructor with property tree configuration
      LinearSolver(const sm::PropertyTree& config);
      /// Copy constructor
      LinearSolver(const Self& other) = delete;
      /// Copy assignment operator
      LinearSolver& operator = (const Self& other) = delete;
      /// Move constructor
      LinearSolver(Self&& other) = delete;
      /// Move assignment operator
      LinearSolver& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~LinearSolver();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Build the system of equations assuming things have been set
      virtual void buildSystem(size_t numThreads, bool useMEstimator);
      /// Solve the system of equations assuming things have been set
      virtual bool solveSystem(Eigen::VectorXd& x);
      /// Helper function for dog leg implementation / steepest descent solution
      virtual double rhsJtJrhs();
      /** 
       * This function solves a system of equation using marginalization.
       * \brief Marginalization solver
       * 
       * \return void
       * \param[in] A sparse matrix left-hand side
       * \param[in] b dense vector right-hand side
       * \param[in] j starting column index for the marginalization
       * \param[out] x solution
       */
      void solve(cholmod_sparse* A, cholmod_dense* b, std::ptrdiff_t j,
        Eigen::VectorXd& x);
      /// Clear the existing symbolic factorization
      void clearFactorization();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the options
      const Options& getOptions() const;
      /// Returns the options
      Options& getOptions();
      /// Returns the name of the solver
      virtual std::string name() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Initialize the matrix structure for the problem
      virtual void initMatrixStructureImplementation(const
        std::vector<aslam::backend::DesignVariable*>& dvs, const
        std::vector<aslam::backend::ErrorTerm*>& errors, bool
        useDiagonalConditioner);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Linear solver options
      Options _options;
      /// Cholmod common structure
      cholmod_common _cholmod;
      /// Caching factorization if needed
      SuiteSparseQR_factorization<double>* _factor;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H
