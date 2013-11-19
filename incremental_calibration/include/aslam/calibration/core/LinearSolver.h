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

#include <aslam/backend/CompressedColumnJacobianTransposeBuilder.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>

template <typename Entry> struct SuiteSparseQR_factorization;

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;
    template<typename I> class CompressedColumnMatrix;

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
            epsSVD(std::numeric_limits<double>::epsilon()),
            epsQR(std::numeric_limits<double>::epsilon()),
            svdTol(-1),
            qrTol(-1),
            verbose(false) {}
        /// Perform column scaling/normalization
        bool columnScaling;
        /// Epsilon for when to consider an element being zero in the norm
        double epsNorm;
        /// Epsilon for SVD numerical rank
        double epsSVD;
        /// Epsilon for QR tolerance computation
        double epsQR;
        /// Fixed tolerance for SVD numerical rank
        double svdTol;
        /// Fixed tolerance for QR
        double qrTol;
        /// Verbose mode
        bool verbose;
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
      virtual bool solveSystem(Eigen::VectorXd& dx);
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
      /** 
       * This function analyzes the marginalized matrix at index j. The goal
       * is to estimate the numerical null and column space, the covariance, the
       * covariance of the column space projected system. All the results are
       * cached in the solver.
       * \brief Marginalization analyzer
       * 
       * \return void
       * \param[in] A sparse matrix left-hand side
       * \param[in] j starting column index for the marginalization
       */
      void analyzeMarginal(cholmod_sparse* A, std::ptrdiff_t j);
      /** 
       * This function does the same as the previous one, except that it uses
       * the stored elements.
       * \brief Marginalization analyzer
       * 
       * \return status
       */
      bool analyzeMarginal();
      /// Clear the cached variables
      void clear();
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
      /// Returns the current SVD rank
      std::ptrdiff_t getSVDRank() const;
      /// Returns the current SVD rank deficiency
      std::ptrdiff_t getSVDRankDeficiency() const;
      /// Returns the current gap in the singular values at the rank
      double getSvGap() const;
      /// Returns the current QR rank
      std::ptrdiff_t getQRRank() const;
      /// Returns the current QR rank deficiency
      std::ptrdiff_t getQRRankDeficiency() const;
      /// Returns the marginalization start index
      std::ptrdiff_t getMargStartIndex() const;
      /// Sets the marginalization start index
      void setMargStartIndex(std::ptrdiff_t index);
      /// Returns the current Jacobian transpose
      const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
        getJacobianTranspose() const;
      /// Returns the current tolerance used by SPQR
      double getQRTolerance() const;
      /// Returns the current tolerance used by SVD
      double getSVDTolerance() const;
      /// Returns the current singular values
      const Eigen::VectorXd& getSingularValues() const;
      /// Returns the current U matrix of SVD
      const Eigen::MatrixXd& getMatrixU() const;
      /// Returns the current null space for SVD
      Eigen::MatrixXd getNullSpace() const;
      /// Returns the current column space for SVD
      Eigen::MatrixXd getColumnSpace() const;
      /// Returns the current covariance matrix for SVD
      Eigen::MatrixXd getCovariance() const;
      /// Returns the current projected covariance matrix for SVD
      Eigen::MatrixXd getProjectedCovariance() const;
      /// Returns the current log2 sum of the singular values
      double getSingularValuesLog2Sum() const;
      /// Returns the peak memory usage in bytes
      size_t getPeakMemoryUsage() const;
      /// Returns the current memory usage in bytes
      size_t getMemoryUsage() const;
      /// Returns the current number of flops
      double getNumFlops() const;
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
      /// Caching current factorization if needed
      SuiteSparseQR_factorization<double>* _factor;
      /// Caching current estimated numerical rank for SVD
      std::ptrdiff_t _svdRank;
      /// Caching current gap in estimated singular values at the rank
      double _svGap;
      /// Caching current estimated numerical rank deficiency for SVD
      std::ptrdiff_t _svdRankDeficiency;
      /// Jacobian builder
      aslam::backend::CompressedColumnJacobianTransposeBuilder<std::ptrdiff_t>
        _jacobianBuilder;
      /// Marginalization start index
      std::ptrdiff_t _margStartIndex;
      /// Caching current SVD tolerance
      double _svdTolerance;
      /// Caching current singular values
      Eigen::VectorXd _singularValues;
      /// Caching current U matrix of SVD
      Eigen::MatrixXd _matrixU;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H
