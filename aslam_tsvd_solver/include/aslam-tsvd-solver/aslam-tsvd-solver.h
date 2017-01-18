#ifndef ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H
#define ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H

#include <cstddef>
#include <string>
#include <vector>

#include <aslam/backend/CompressedColumnJacobianTransposeBuilder.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <Eigen/Core>

#include <truncated-svd-solver/tsvd-solver.h>
#include <truncated-svd-solver/tsvd-solver-options.h>

template<typename Entry> struct SuiteSparseQR_factorization;

namespace sm {
class ConstPropertyTree;
}

namespace aslam {
namespace backend {
class DesignVariable;
class ErrorTerm;
template<typename I> class CompressedColumnMatrix;
}

namespace backend {
/** The class AslamTruncatedSvdSolver interfaces the truncated_svd_solver
 *  and provides the necessary interfaces to work as an aslam solver.
 */
class AslamTruncatedSvdSolver
    : public aslam::backend::LinearSystemSolver,
      public truncated_svd_solver::TruncatedSvdSolver {
 public:
  typedef truncated_svd_solver::TruncatedSvdSolverOptions Options;
  /// Constructor with options structure
  AslamTruncatedSvdSolver(const Options& options = Options());
  /// Constructor with property tree configuration
  AslamTruncatedSvdSolver(const sm::ConstPropertyTree& config);
  /// Copy constructor
  AslamTruncatedSvdSolver(const AslamTruncatedSvdSolver& other) = delete;
  /// Copy assignment operator
  AslamTruncatedSvdSolver& operator= (
      const AslamTruncatedSvdSolver& other) = delete;
  /// Move constructor
  AslamTruncatedSvdSolver(AslamTruncatedSvdSolver&& other) = delete;
  /// Move assignment operator
  AslamTruncatedSvdSolver& operator= (
      AslamTruncatedSvdSolver&& other) = delete;
  /// Destructor
  virtual ~AslamTruncatedSvdSolver();

  /// Build the system of equations assuming things have been set
  virtual void buildSystem(size_t numThreads, bool useMEstimator) override;
  /// Solve the system of equations assuming things have been set
  virtual bool solveSystem(Eigen::VectorXd& dx) override;
  /// Helper function for dog leg implementation / steepest descent solution
  virtual double rhsJtJrhs() override;

  virtual std::string name() const override {
    return std::string("marginal_spqr_svd");
  }

  bool analyzeMarginal();
  const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
      getJacobianTranspose() const;

 protected:
  /// Initialize the matrix structure for the problem
  virtual void initMatrixStructureImplementation(
      const std::vector<aslam::backend::DesignVariable*>& dvs,
      const std::vector<aslam::backend::ErrorTerm*>& errors,
      bool use_diagonal_conditioner);

  virtual void handleNewAcceptConstantErrorTerms() override;

 private:
  aslam::backend::CompressedColumnJacobianTransposeBuilder<std::ptrdiff_t>
    jacobian_builder_;
};

}  // namespace backend
}  // namespace aslam

#endif // ASLAM_CALIBRATION_CORE_LINEAR_SOLVER_H
