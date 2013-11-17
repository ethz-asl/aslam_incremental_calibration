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

#include "aslam/calibration/core/LinearSolver.h"

#include <algorithm>

#include <SuiteSparseQR.hpp>

#include <sm/PropertyTree.hpp>

#include "aslam/calibration/algorithms/linalg.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    LinearSolver::LinearSolver(const Options& options) :
        _options(options),
        _factor(NULL) {
      cholmod_l_start(&_cholmod);
    }

    LinearSolver::LinearSolver(const sm::PropertyTree& config) :
        _factor(NULL) {
      cholmod_l_start(&_cholmod);
    }

    LinearSolver::~LinearSolver() {
      clearFactorization();
      cholmod_l_finish(&_cholmod);
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const LinearSolver::Options& LinearSolver::getOptions() const {
      return _options;
    }

    LinearSolver::Options& LinearSolver::getOptions() {
      return _options;
    }

    std::string LinearSolver::name() const {
      return std::string("marginal_spqr_svd");
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void LinearSolver::buildSystem(size_t numThreads, bool useMEstimator) {
    }

    bool LinearSolver::solveSystem(Eigen::VectorXd& x) {
    }

    double LinearSolver::rhsJtJrhs() {
    }

    void LinearSolver::initMatrixStructureImplementation(const
        std::vector<aslam::backend::DesignVariable*>& dvs, const
        std::vector<aslam::backend::ErrorTerm*>& errors, bool
        useDiagonalConditioner) {
    }

    void LinearSolver::solve(cholmod_sparse* A, cholmod_dense* b,
        std::ptrdiff_t j, Eigen::VectorXd& x) {
      if (A->nrow != b->nrow)
        throw InvalidOperationException("LinearSolver::solve(): "
          "inconsistent A and b");
      cholmod_sparse* A_l = columnSubmatrix(A, 0, j - 1, &_cholmod);
      cholmod_dense* G_l = NULL;
      if (_options.columnScaling) {
        try {
          G_l = columnScalingMatrix(A_l, &_cholmod, _options.epsNorm);
        }
        catch (...) {
          cholmod_l_free_sparse(&A_l, &_cholmod);
          throw;
        }
        if (!cholmod_l_scale(G_l, CHOLMOD_COL, A_l, &_cholmod)) {
          cholmod_l_free_dense(&G_l, &_cholmod);
          cholmod_l_free_sparse(&A_l, &_cholmod);
          throw InvalidOperationException("LinearSolver::solve(): "
            "cholmod_l_scale failed");
        }
      }
      if (_factor && _factor->QRsym &&
          (_factor->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
          _factor->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol)))
        clearFactorization();
      if (!_factor) {
        _factor = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
          SPQR_DEFAULT_TOL, A_l, &_cholmod);
        if (_factor == NULL) {
          cholmod_l_free_sparse(&A_l, &_cholmod);
          if (G_l)
            cholmod_l_free_dense(&G_l, &_cholmod);
          throw InvalidOperationException("LinearSolver::solve(): "
            "SuiteSparseQR_symbolic failed");
        }
      }
      // TODO: add tolerance here if needed
      const int status = SuiteSparseQR_numeric<double>(qrTol(A_l,
       _options.epsQR), A_l, _factor, &_cholmod);
      cholmod_l_free_sparse(&A_l, &_cholmod);
      if (!status) {
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        throw InvalidOperationException("LinearSolver::solve(): "
          "SuiteSparseQR_numeric failed");
      }
      cholmod_sparse* A_r = columnSubmatrix(A, j, A->ncol - 1, &_cholmod);
      cholmod_dense* G_r = NULL;
      if (_options.columnScaling) {
        try {
          G_r = columnScalingMatrix(A_r, &_cholmod, _options.epsNorm);
        }
        catch (...) {
          cholmod_l_free_sparse(&A_r, &_cholmod);
          cholmod_l_free_dense(&G_l, &_cholmod);
          throw;
        }
        if (!cholmod_l_scale(G_r, CHOLMOD_COL, A_r, &_cholmod)) {
          cholmod_l_free_sparse(&A_r, &_cholmod);
          cholmod_l_free_dense(&G_r, &_cholmod);
          cholmod_l_free_dense(&G_l, &_cholmod);
          throw InvalidOperationException("LinearSolver::solve(): "
            "cholmod_l_scale failed");
        }
      }
      cholmod_sparse* A_rt = cholmod_l_transpose(A_r, 1, &_cholmod);
      if (A_rt == NULL) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw InvalidOperationException("LinearSolver::solve(): "
          "cholmod_l_transpose failed");
      }
      cholmod_sparse* Omega = NULL;
      cholmod_sparse* A_rtQ = NULL;
      try {
        reduceLeftHandSide(_factor, A_rt, &Omega, &A_rtQ, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        cholmod_l_free_sparse(&A_rt, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      Eigen::VectorXd sv;
      Eigen::MatrixXd U;
      analyzeSVD(Omega, sv, U);
      cholmod_l_free_sparse(&Omega, &_cholmod);
      cholmod_dense* b_r;
      try {
        b_r = reduceRightHandSide(_factor, A_rt, A_rtQ, b, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        cholmod_l_free_sparse(&A_rt, &_cholmod);
        cholmod_l_free_sparse(&A_rtQ, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      cholmod_l_free_sparse(&A_rt, &_cholmod);
      cholmod_l_free_sparse(&A_rtQ, &_cholmod);
      std::ptrdiff_t nrank = estimateNumericalRank(sv, rankTol(sv,
        _options.epsRank));
      Eigen::VectorXd x_r;
      solveSVD(b_r, sv, U, nrank, x_r);
      cholmod_l_free_dense(&b_r, &_cholmod);
      cholmod_dense* x_l;
      try {
        x_l = solveQR(_factor, b, A_r, x_r, &_cholmod);
      }
      catch (...) {
        cholmod_l_free_sparse(&A_r, &_cholmod);
        if (G_l)
          cholmod_l_free_dense(&G_l, &_cholmod);
        if (G_r)
          cholmod_l_free_dense(&G_r, &_cholmod);
        throw;
      }
      if (_options.columnScaling) {
        const double* G_l_val = reinterpret_cast<const double*>(G_l->x);
        double* x_l_val = reinterpret_cast<double*>(x_l->x);
        for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(x_l->nrow);
            ++i)
          x_l_val[i] = G_l_val[i] * x_l_val[i];
        cholmod_l_free_dense(&G_l, &_cholmod);
        Eigen::Map<const Eigen::VectorXd> G_rEigen(
          reinterpret_cast<const double*>(G_r->x), G_r->nrow);
        x_r = G_rEigen.array() * x_r.array();
        cholmod_l_free_dense(&G_r, &_cholmod);
      }
      cholmod_l_free_sparse(&A_r, &_cholmod);
      x.resize(A->ncol);
      const double* x_l_val = reinterpret_cast<const double*>(x_l->x);
      std::copy(x_l_val, x_l_val + x_l->nzmax, x.data());
      cholmod_l_free_dense(&x_l, &_cholmod);
      x.tail(x_r.size()) = x_r;
    }

    void LinearSolver::clearFactorization() {
      if (_factor) {
        SuiteSparseQR_free<double>(&_factor, &_cholmod);
        _factor = NULL;
      }
    }

  }
}
