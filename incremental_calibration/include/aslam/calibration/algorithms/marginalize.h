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

/** \file marginalize.h
    \brief This file defines functions for the marginalization of variables.
  */

#ifndef ASLAM_CALIBRATION_ALGORITHMS_MARGINALIZE_H
#define ASLAM_CALIBRATION_ALGORITHMS_MARGINALIZE_H

#include <cstdlib>

#include <Eigen/Core>

#include <cholmod.h>

namespace aslam {
  namespace backend {

    template<typename I> class CompressedColumnMatrix;

  }
  namespace calibration {

    /** \name Methods
      @{
      */
    /** 
     * This function marginalizes variables from a sparse Jacobian.
     * \brief Variables marginalization
     * 
     * \param[in] Jt Jacobian transpose as outputted by linear solvers
     * \param[in] j index from where to marginalize
     * \param[in] normTol tolerance for a zero norm column
     * \param[in] epsTol tolerance for SVD tolerance computation
     */
    void marginalize(const aslam::backend::CompressedColumnMatrix<ssize_t>& Jt,
      size_t j, double normTol = 1e-8, double epsTol = 1e-4);

    /** 
     * This function returns the marginal Jacobian from two submatrices.
     * \brief Marginal Jacobian recovery
     * 
     * \return marginal Jacobian
     * \param[in] J_x is the Jacobian containing the state variables
     * \param[in] J_thetat is the tranposed Jacobian containing the calibration
     *            variables
     */
    Eigen::MatrixXd marginalJacobian(cholmod_sparse* J_x, cholmod_sparse*
      J_thetat, cholmod_common* cholmod);

    /** 
     * This function returns the column 2-norm of a sparse matrix.
     * \brief Sparse matrix column 2-norm
     * 
     * \return 2-norm of the specified column
     * \param[in] A sparse matrix
     * \param[in] j column index
     */
    double colNorm(cholmod_sparse* A, size_t j);
    /** @}
      */

  }
}

#endif // ASLAM_CALIBRATION_ALGORITHMS_MARGINALIZE_H
