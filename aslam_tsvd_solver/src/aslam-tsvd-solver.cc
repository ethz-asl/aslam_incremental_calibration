#include "aslam-tsvd-solver/aslam-tsvd-solver.h"

#include <algorithm>
#include <cmath>

#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <cholmod.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <sm/PropertyTree.hpp>
#include <truncated-svd-solver/cholmod-helpers.h>
#include <truncated-svd-solver/linear-algebra-helpers.h>

namespace aslam {
namespace backend {

AslamTruncatedSvdSolver::Options createTsvdOptionsFromPropertyTree(
    const sm::PropertyTree& config) {
  AslamTruncatedSvdSolver::Options tsvd_options;
  tsvd_options.columnScaling =
      config.getBool("columnScaling", tsvd_options.columnScaling);
  tsvd_options.epsNorm = config.getDouble("epsNorm", tsvd_options.epsNorm);
  tsvd_options.epsSVD = config.getDouble("epsSVD", tsvd_options.epsSVD);
  tsvd_options.epsQR = config.getDouble("epsQR", tsvd_options.epsQR);
  tsvd_options.svdTol = config.getDouble("svdTol", tsvd_options.svdTol);
  tsvd_options.qrTol = config.getDouble("qrTol", tsvd_options.qrTol);
  tsvd_options.verbose = config.getBool("verbose", tsvd_options.verbose);
  return tsvd_options;
}

AslamTruncatedSvdSolver::AslamTruncatedSvdSolver(const Options& options)
    : truncated_svd_solver::TruncatedSvdSolver(options) {}

AslamTruncatedSvdSolver::AslamTruncatedSvdSolver(const sm::PropertyTree& config)
    : AslamTruncatedSvdSolver(createTsvdOptionsFromPropertyTree(config)) {}

AslamTruncatedSvdSolver::~AslamTruncatedSvdSolver() {}

void AslamTruncatedSvdSolver::buildSystem(size_t numThreads,
                                          bool useMEstimator) {
  jacobian_builder_.buildSystem(numThreads, useMEstimator);
}

bool AslamTruncatedSvdSolver::solveSystem(Eigen::VectorXd& dx) {
  aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>& Jt =
    jacobian_builder_.J_transpose();
  cholmod_sparse Jt_CS;
  Jt.getView(&Jt_CS);
  cholmod_sparse* J_CS = cholmod_l_transpose(&Jt_CS, 1, &cholmod_);
  if (J_CS == NULL)
    return false;
  cholmod_dense e_CD;
  truncated_svd_solver::eigenDenseToCholmodDenseView(_e, &e_CD);
  bool status = true;
  solve(J_CS, &e_CD, margStartIndex_, dx);
  if (tsvd_options_.verbose) {
    std::cout << "SVD rank: " << getSVDRank() << std::endl;
    std::cout << "SVD rank deficiency: " << getSVDRankDeficiency()
      << std::endl;
    std::cout << "QR rank: " << getQRRank() << std::endl;
    std::cout << "QR rank deficiency: " << getQRRankDeficiency()
      << std::endl;
    std::cout.precision(3);
    std::cout.width(4);
    std::cout << "Singular values:\n" << getSingularValues().transpose()
      << std::endl;
    std::cout << "V-matrix (observability basis, column vectors correspond to singular values)\n" << getMatrixV()
      << std::endl;
  }
  cholmod_l_free_sparse(&J_CS, &cholmod_);
  return status;
}

double AslamTruncatedSvdSolver::rhsJtJrhs() {
  return 0.0;
}

void AslamTruncatedSvdSolver::initMatrixStructureImplementation(const
    std::vector<aslam::backend::DesignVariable*>& dvs, const
    std::vector<aslam::backend::ErrorTerm*>& errors, bool
    useDiagonalConditioner) {
  CHECK(!useDiagonalConditioner) << "useDiagonalConditioner not supported in AslamTruncatedSvdSolver";
  clear();
  jacobian_builder_.initMatrixStructure(dvs, errors);
}

bool AslamTruncatedSvdSolver::analyzeMarginal() {
  aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>& Jt =
      jacobian_builder_.J_transpose();
  cholmod_sparse Jt_CS;
  Jt.getView(&Jt_CS);
  cholmod_common cholmod;
  truncated_svd_solver::SelfFreeingCholmodPtr<cholmod_sparse> J_CS(
      cholmod_l_transpose(&Jt_CS, 1, &cholmod), cholmod);
  if (J_CS == nullptr) {
    return false;
  }
  truncated_svd_solver::TruncatedSvdSolver::analyzeMarginal(
      J_CS, margStartIndex_);
  return true;
}

const aslam::backend::CompressedColumnMatrix<std::ptrdiff_t>&
  AslamTruncatedSvdSolver::getJacobianTranspose() const {
  return jacobian_builder_.J_transpose();
}

}  // namespace backend
}  // namespace aslam
