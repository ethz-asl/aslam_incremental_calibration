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

/** \file IncrementalEstimator.h
    \brief This file defines the IncrementalEstimator class, which implements
           an incremental estimator for robotic calibration problems.
  */

#ifndef ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H
#define ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

namespace aslam {
  namespace backend {

    class SparseQrLinearSystemSolver;

  }
  namespace calibration {

    class OptimizationProblem;
    class IncrementalOptimizationProblem;

    /** The class IncrementalEstimator implements an incremental estimator
        for robotic calibration problems.
        \brief Incremental estimator
      */
    class IncrementalEstimator {
    public:
      /** \name Types definitions
        @{
        */
      /// Optimization problem type (shared pointer)
      typedef boost::shared_ptr<OptimizationProblem> Batch;
      /// Incremental optimization problem (shared pointer)
      typedef boost::shared_ptr<IncrementalOptimizationProblem>
        IncrementalOptimizationProblemSP;
      /// Self type
      typedef IncrementalEstimator Self;
      /// Solver type
      typedef aslam::backend::SparseQrLinearSystemSolver LinearSolver;
      /// Options for the incremental estimator
      struct Options {
        /// Mutual information threshold
        double _miTol;
        /// QR treshold for rank-deficiency
        double _qrTol;
        /// Verbosity of the optimizer
        bool _verbose;
        /// Perform column normalization
        bool _colNorm;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs estimator with group to marginalize and options
      IncrementalEstimator(size_t groupId,
        const Options& options = _defaultOptions);
      /// Copy constructor
      IncrementalEstimator(const Self& other) = delete;
      /// Copy assignment operator
      IncrementalEstimator& operator = (const Self& other) = delete;
      /// Move constructor
      IncrementalEstimator(Self&& other) = delete;
      /// Move assignment operator
      IncrementalEstimator& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~IncrementalEstimator();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Add a measurement batch to the estimator
      void addBatch(const Batch& batch, bool force = false);
      /// Remove a measurement batch from the estimator
      void removeBatch(size_t idx);
      /// Returns the covariance matrix of the marginalized variables
      Eigen::MatrixXd getMarginalizedCovariance() const;
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the incremental optimization problem
      const IncrementalOptimizationProblem* getProblem() const;
      /// Returns the current options
      const Options& getOptions() const;
      /// Returns the current options
      Options& getOptions();
      /// Returns the last computed mutual information
      double getMutualInformation() const;
      /// Return the marginalized group ID
      size_t getMargGroupId() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Runs an optimization with current setup
      double optimize();
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Underlying optimization problem
      IncrementalOptimizationProblemSP _problem;
      /// Group ID to marginalize
      size_t _margGroupId;
      /// Mutual information
      double _mi;
      /// Sum of the log of the diagonal elements of R
      double _sumLogDiagR;
      /// Options
      Options _options;
      /// Default options
      static const struct Options _defaultOptions;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H
