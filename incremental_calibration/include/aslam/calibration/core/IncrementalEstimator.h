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

#include <cstddef>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

#include <aslam/backend/Optimizer2Options.hpp>

#include "aslam/calibration/core/LinearSolverOptions.h"

namespace sm {

  class PropertyTree;

}
namespace aslam {
  namespace backend {

    class GaussNewtonTrustRegionPolicy;
    class Optimizer2;
    template<typename I> class CompressedColumnMatrix;

  }
  namespace calibration {

    class OptimizationProblem;
    class IncrementalOptimizationProblem;
    class LinearSolver;

    /** The class IncrementalEstimator implements an incremental estimator
        for robotic calibration problems.
        \brief Incremental estimator
      */
    class IncrementalEstimator {
    public:
      /** \name Types definitions
        @{
        */
      /// Optimization problem type
      typedef OptimizationProblem Batch;
      /// Optimization problem type (shared pointer)
      typedef boost::shared_ptr<OptimizationProblem> BatchSP;
      /// Incremental optimization problem (shared pointer)
      typedef boost::shared_ptr<IncrementalOptimizationProblem>
        IncrementalOptimizationProblemSP;
      /// Self type
      typedef IncrementalEstimator Self;
      /// Trust region type
      typedef aslam::backend::GaussNewtonTrustRegionPolicy TrustRegionPolicy;
      /// Optimizer type
      typedef aslam::backend::Optimizer2 Optimizer;
      /// Optimizer options type
      typedef aslam::backend::Optimizer2Options OptimizerOptions;
      /// Optimizer type (shared_ptr)
      typedef boost::shared_ptr<Optimizer> OptimizerSP;
      /// Options for the incremental estimator
      struct Options {
        Options() :
            miTol(0.5),
            verbose(false) {
        }
        /// Mutual information threshold
        double miTol;
        /// Verbosity of the estimator
        bool verbose;
      };
      /// Return value when adding a batch
      struct ReturnValue {
        /// True if the batch was accepted
        bool batchAccepted;
        /// Mutual information that the batch contributed
        double mutualInformation;
        /// Rank that this batch lead
        std::ptrdiff_t rank;
        /// Rank deficiency that this batch lead
        std::ptrdiff_t rankDeficiency;
        /// Marginal rank that this batch lead
        std::ptrdiff_t marginalRank;
        /// Marginal rank deficiency that this batch lead
        std::ptrdiff_t marginalRankDeficiency;
        /// SVD tolerance used for this batch
        double svdTolerance;
        /// QR tolerance used for this batch
        double qrTolerance;
        /// Null space of the marginalized system
        Eigen::MatrixXd nullSpace;
        /// Column space of the marginalized system
        Eigen::MatrixXd columnSpace;
        /// Covariance of the marginalized system
        Eigen::MatrixXd covariance;
        /// Projected covariance of the marginalized system
        Eigen::MatrixXd projectedCovariance;
        /// Singular values of the marginalized system
        Eigen::VectorXd singularValues;
        /// Scaled singular values of the marginalized system
        Eigen::VectorXd scaledSingularValues;
        /// Number of iterations
        size_t numIterations;
        /// Cost function at start
        double JStart;
        /// Cost function at end
        double JFinal;
        /// Elapsed time for processing this batch [s]
        double elapsedTime;
      };
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs estimator with group to marginalize and options
      IncrementalEstimator(size_t groupId, const Options& options = Options(),
        const LinearSolverOptions& linearSolverOptions = LinearSolverOptions(),
        const OptimizerOptions& optimizerOptions = OptimizerOptions());
      /// Constructs estimator with configuration in property tree
      IncrementalEstimator(const sm::PropertyTree& config);
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
      /// Adds a measurement batch to the estimator
      ReturnValue addBatch(const BatchSP& batch, bool force = false);
      /// Removes a measurement batch from the estimator
      void removeBatch(size_t idx);
      /// Removes a measurement batch from the estimator
      void removeBatch(const BatchSP& batch);
      /// The number of batches
      size_t getNumBatches() const;
      /// Re-runs the optimizer
      ReturnValue reoptimize();
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
      /// Returns the linear solver options
      const LinearSolverOptions& getLinearSolverOptions() const;
      /// Returns the linear solver options
      LinearSolverOptions& getLinearSolverOptions();
      /// Returns the optimizer options
      const OptimizerOptions& getOptimizerOptions() const;
      /// Returns the optimizer options
      OptimizerOptions& getOptimizerOptions();
      /// Return the marginalized group ID
      size_t getMargGroupId() const;
      /// Returns the last computed mutual information
      double getMutualInformation() const;
      /// Returns the current Jacobian transpose if available
      const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
        getJacobianTranspose() const;
      /// Returns the current estimated non marginal numerical rank
      std::ptrdiff_t getRank() const;
      /// Returns the current estimated non marginal numerical rank deficiency
      std::ptrdiff_t getRankDeficiency() const;
      /// Returns the current estimated marginal numerical rank
      std::ptrdiff_t getMarginalRank() const;
      /// Returns the current estimated marginal numerical rank deficiency
      std::ptrdiff_t getMarginalRankDeficiency() const;
      /// Returns the current tolerance for the SVD decomposition
      double getSVDTolerance() const;
      /// Returns the current tolerance for the QR decomposition
      double getQRTolerance() const;
      /// Returns the current marginalized null space
      const Eigen::MatrixXd& getMarginalizedNullSpace() const;
      /// Returns the current marginalized column space
      const Eigen::MatrixXd& getMarginalizedColumnSpace() const;
      /// Returns the current marginalized covariance
      const Eigen::MatrixXd& getMarginalizedCovariance() const;
      /// Returns the current projected marginalized covariance
      const Eigen::MatrixXd& getProjectedMarginalizedCovariance() const;
      /// Returns the current singular values of the marginalized system
      const Eigen::VectorXd& getSingularValues() const;
      /// Returns the current scaled singular values if scaling enabled
      const Eigen::VectorXd& getScaledSingularValues() const;
      /// Returns the peak memory usage in bytes
      size_t getPeakMemoryUsage() const;
      /// Returns the current memory usage in bytes
      size_t getMemoryUsage() const;
      /// Returns the number of flops of the linear solver
      double getNumFlops() const;
      /// Returns the current initial cost for the estimator
      double getInitialCost() const;
      /// Returns the current final cost for the estimator
      double getFinalCost() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Ensures the marginalized variables are well located
      void orderMarginalizedDesignVariables();
      /// Restores the linear solver
      void restoreLinearSolver();
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Options
      Options _options;
      /// Group ID to marginalize
      size_t _margGroupId;
      /// Underlying optimizer
      OptimizerSP _optimizer;
      /// Underlying optimization problem
      IncrementalOptimizationProblemSP _problem;
      /// Mutual information
      double _mutualInformation;
      /// Sum of the log2 of the singular values up to the numerical rank
      double _svLog2Sum;
      /// Null space of the marginalized system
      Eigen::MatrixXd _nullSpace;
      /// Column space of the marginalized system
      Eigen::MatrixXd _columnSpace;
      /// Covariance of the marginalized system
      Eigen::MatrixXd _covariance;
      /// Projected covariance of the marginalized system
      Eigen::MatrixXd _projectedCovariance;
      /// Singular values of the marginalized system
      Eigen::VectorXd _singularValues;
      /// Scaled singular values of the marginalized system if scaling enabled
      Eigen::VectorXd _scaledSingularValues;
      /// Tolerance for SVD
      double _svdTolerance;
      /// Tolerance for QR
      double _qrTolerance;
      /// Estimated numerical rank of the marginalized system
      std::ptrdiff_t _svdRank;
      /// Estimated numerical rank deficiency of the marginalized system
      std::ptrdiff_t _svdRankDeficiency;
      /// Estimated numerical rank of the non-marginalized system
      std::ptrdiff_t _qrRank;
      /// Estimated numerical rank deficiency of the non-marginalized system
      std::ptrdiff_t _qrRankDeficiency;
      /// Peak memory usage
      size_t _peakMemoryUsage;
      /// Memory usage
      size_t _memoryUsage;
      /// Number of flops
      double _numFlops;
      /// Initial cost
      double _initialCost;
      /// Final cost
      double _finalCost;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_INCREMENTAL_ESTIMATOR_H
