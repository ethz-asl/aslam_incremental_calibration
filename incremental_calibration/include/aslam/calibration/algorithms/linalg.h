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

/** \file linalg.h
    \brief This file defines useful functions for linear algebra operations.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_LINALG_H
#define ASLAM_CALIBRATION_ALGORITHMS_LINALG_H

#include <cstddef>

#include <limits>

#include <Eigen/Core>

struct cholmod_sparse_struct;
typedef cholmod_sparse_struct cholmod_sparse;
struct cholmod_dense_struct;
typedef cholmod_dense_struct cholmod_dense;
struct cholmod_common_struct;
typedef cholmod_common_struct cholmod_common;
template <typename Entry> struct SuiteSparseQR_factorization;

namespace aslam {
  namespace calibration {

    /** \name Methods
      @{
      */
      /** 
       * This function extracts a submatrix based on column indices from a
       * sparse matrix.
       * \brief Submatrix column extraction
       * 
       * \return column submatrix pointer (owned by caller)
       * \param[in] A sparse matrix
       * \param[in] colStartIdx starting column index
       * \param[in] colEndIdx ending column index
       * \param[in] cholmod cholmod common structure
       */
      cholmod_sparse* columnSubmatrix(cholmod_sparse* A, std::ptrdiff_t
        colStartIdx, std::ptrdiff_t colEndIdx, cholmod_common* cholmod);
      /** 
       * This function extracts a submatrix based on row indices from a
       * sparse matrix.
       * \brief Submatrix row extraction
       * 
       * \return row submatrix pointer (owned by caller)
       * \param[in] A sparse matrix
       * \param[in] rowStartIdx starting row index
       * \param[in] rowEndIdx ending row index
       * \param[in] cholmod cholmod workspace
       */
      cholmod_sparse* rowSubmatrix(cholmod_sparse* A, std::ptrdiff_t
        rowStartIdx, std::ptrdiff_t rowEndIdx, cholmod_common* cholmod);
      /** 
       * This function returns the column 2-norm of a sparse matrix.
       * \brief Sparse matrix column 2-norm
       * 
       * \return 2-norm of the specified column
       * \param[in] A sparse matrix
       * \param[in] j column index
       */
      double colNorm(const cholmod_sparse* A, std::ptrdiff_t j);
      /** 
       * This function returns the column normalization scaling matrix.
       * According to Van der Sluis (1969), this matrix minimizes the condition
       * number of A.
       * \brief Column normalization scaling matrix
       * 
       * \return scaling matrix pointer (owned by caller)
       * \param[in] A sparse matrix
       * \param[in] eps tolerance for when to consider an element zero
       * \param[in] cholmod cholmod workspace
       */
      cholmod_dense* columnScalingMatrix(cholmod_sparse* A, cholmod_common*
        cholmod, double eps = std::numeric_limits<double>::epsilon());
      /** 
       * This function views an Eigen vector as a cholmod_dense vector.
       * \brief Eigen vector to cholmod_dense vector view
       * 
       * \return void
       * \param[in] in Eigen dense vector
       * \param[out] out cholmod dense vector
       */
      void eigenDenseToCholmodDenseView(const Eigen::VectorXd& in,
        cholmod_dense* out);
      /** 
       * This function copies a cholmod_sparse matrix into an Eigen dense
       * matrix.
       * \brief Cholmod sparse matrix to Eigen dense converter
       * 
       * \return void
       * \param[in] in cholmod sparse matrix
       * \param[out] out Eigen dense matrix
       */
      void cholmodSparseToEigenDenseCopy(const cholmod_sparse* in,
        Eigen::MatrixXd& out);
      /** 
       * This function computes the numerical rank of a matrix based on
       * a vector of its singular values.
       * \brief Numeric rank estimate
       * 
       * \return numerical rank
       * \param[in] sv vector of singular values
       * \param[in] tol tolerance for numerical rank determination
       */
      std::ptrdiff_t estimateNumericalRank(const Eigen::VectorXd& sv, double
        tol);
      /** 
       * This function computes a numerical rank tolerance.
       * \brief Numeric rank tolerance estimation
       * 
       * \return numerical rank tolerance
       * \param[in] sv vector of singular values
       * \param[in] eps machine precision
       */
      double rankTol(const Eigen::VectorXd& sv, double eps =
        std::numeric_limits<double>::epsilon());
      /** 
       * This function returns the matrix where the elements on the left side
       * have been marginalized out.
       * \brief Reduced left-hand side recover
       * 
       * \return void
       * \param[in] factor QR factorization of the left side of the original
       *            matrix
       * \param[in] A_rt transpose of the right side of the original matrix
       * \param[in] cholmod cholmod workspace
       * \param[out] Omega reduced matrix (owned by caller)
       * \param[out] A_rtQ A_r' * Q, used subsequently by reduceRightHandSide
       *             (owned by caller)
       */
       void reduceLeftHandSide(SuiteSparseQR_factorization<double>*
        factor, cholmod_sparse* A_rt, cholmod_sparse* Omega, cholmod_sparse*
        A_rtQ, cholmod_common* cholmod);
      /** 
       * This function returns the reduced right-hand side of a system.
       * \brief Reduced right-hand side recovery
       * 
       * \return reduced right-hand side pointer (owned by caller)
       * \param[in] factor QR factorization of the left side of the original
       *            matrix
       * \param[in] A_rt transpose of the right side of the original matrix
       * \param[in] b original right-hand side
       * \param[in] A_rtQ A_r' * Q, comes from reduceLeftHandSide
       * \param[in] cholmod cholmod workspace
       */
      cholmod_dense* reduceRightHandSide(SuiteSparseQR_factorization<double>*
        factor, cholmod_sparse* A_rt, cholmod_sparse* A_rtQ, cholmod_dense* b,
        cholmod_common* cholmod);
      /** 
       * This function performs SVD analysis on the input matrix which is
       * supposed to be a square symmetric matrix, therefore returning only
       * U, since U=V.
       * \brief SVD analysis
       * 
       * \return void
       * \param[in] Omega matrix to analyze
       * \param[out] sv vector of singular values
       * \param[out] U U part from SVD
       */
      void analyzeSVD(const cholmod_sparse* Omega, Eigen::VectorXd& sv,
        Eigen::MatrixXd& U);
      /** 
       * This function solves a system with its SVD factorization, assuming
       * the system comes from a square symmetric matrix.
       * \brief SVD solver
       * 
       * \return void
       * \param[in] b right-hand side
       * \param[in] sv vector of singular values
       * \param[in] U part from SVD
       * \param[in] rank estimated numerical rank
       * \param[out] x result
       */
      void solveSVD(const cholmod_dense* b, const Eigen::VectorXd& sv,
        const Eigen::MatrixXd& U, std::ptrdiff_t rank, Eigen::VectorXd& x);
      /** 
       * This function solves the rest of the system with the QR factorization.
       * \brief Solver for rest of the system
       * 
       * \return void
       * \param[in] factor QR factorization of the left side of the original
       *            matrix
       * \param[in] b original right-hand side
       * \param[in] A_r right side of the original matrix
       * \param[in] x_r LS solution for the right side of the original matrix
       * \param[in] cholmod cholmod workspace
       * \param[out] x_l QR solution for the left side of the original matrix
       *             (owned by caller)
       */
      void solveQR(SuiteSparseQR_factorization<double>* factor, cholmod_dense*
        b, cholmod_sparse* A_r, const Eigen::VectorXd& x_r, cholmod_dense* x_l,
        cholmod_common* cholmod);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_ALGORITHMS_LINALG_H
