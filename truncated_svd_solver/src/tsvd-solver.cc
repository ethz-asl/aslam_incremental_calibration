#include "truncated-svd-solver/tsvd-solver.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include <glog/logging.h>
#include <SuiteSparseQR.hpp>

#include "truncated-svd-solver/cholmod-helpers.h"
#include "truncated-svd-solver/linear-algebra-helpers.h"
#include "truncated-svd-solver/timing.h"

namespace truncated_svd_solver {

TruncatedSvdSolver::TruncatedSvdSolver(const Options& options) :
    options_(options),
    factor_(nullptr),
    svdRank_(-1),
    svGap_(std::numeric_limits<double>::infinity()),
    svdRankDeficiency_(-1),
    margStartIndex_(-1),
    svdTolerance_(-1.0),
    linearSolverTime_(0.0),
    marginalAnalysisTime_(0.0) {
  cholmod_l_start(&cholmod_);
  cholmod_.SPQR_grain = 16;  // Maybe useless.
}

// TODO(schneith): Move PropertyTree stuff to aslam glue code.
//TruncatedSvdSolver::LinearSolver(const sm::PropertyTree& config) :
//    _factor(nullptr),
//    _svdRank(-1),
//    _svGap(std::numeric_limits<double>::infinity()),
//    _svdRankDeficiency(-1),
//    _margStartIndex(-1),
//    _svdTolerance(-1.0),
//    _linearSolverTime(0.0),
//    _marginalAnalysisTime(0.0) {
//  cholmod_l_start(&_cholmod);
//  _cholmod.SPQR_grain = 16; // maybe useless
//  _options.columnScaling = config.getBool("columnScaling",
//    _options.columnScaling);
//  _options.epsNorm = config.getDouble("epsNorm", _options.epsNorm);
//  _options.epsSVD = config.getDouble("epsSVD", _options.epsSVD);
//  _options.epsQR = config.getDouble("epsQR", _options.epsQR);
//  _options.svdTol = config.getDouble("svdTol", _options.svdTol);
//  _options.qrTol = config.getDouble("qrTol", _options.qrTol);
//  _options.verbose = config.getBool("verbose", _options.verbose);
//  // TODO(schneith): ideally, use constructor delegation, not available for now.
//}

TruncatedSvdSolver::~TruncatedSvdSolver() {
  clear();
  cholmod_l_finish(&cholmod_);
  if (options_.verbose && getMemoryUsage() > 0) {
    LOG(ERROR) << "Cholmod memory leak detected.";
  }
}

void TruncatedSvdSolver::clear() {
  if (factor_) {
    SuiteSparseQR_free<double>(&factor_, &cholmod_);
    factor_ = nullptr;
  }
  clearSvdAnalysisResultMembers();
  linearSolverTime_ = 0.0;
  marginalAnalysisTime_ = 0.0;
}

const TruncatedSvdSolver::Options& TruncatedSvdSolver::getOptions() const {
  return options_;
}

TruncatedSvdSolver::Options& TruncatedSvdSolver::getOptions() {
  return options_;
}

std::string TruncatedSvdSolver::name() const {
  return std::string("marginal_spqr_svd");
}

std::ptrdiff_t TruncatedSvdSolver::getSVDRank() const {
  return svdRank_;
}

std::ptrdiff_t TruncatedSvdSolver::getSVDRankDeficiency() const {
  return svdRankDeficiency_;
}

double TruncatedSvdSolver::getSvGap() const {
  return svGap_;
}

std::ptrdiff_t TruncatedSvdSolver::getQRRank() const {
  if (factor_ && factor_->QRnum)
    return factor_->rank;
  else
    return 0;
}

std::ptrdiff_t TruncatedSvdSolver::getQRRankDeficiency() const {
  if (factor_ && factor_->QRsym && factor_->QRnum)
    return factor_->QRsym->n - factor_->rank;
  else
    return 0;
}

std::ptrdiff_t TruncatedSvdSolver::getMargStartIndex() const {
  return margStartIndex_;
}

void TruncatedSvdSolver::setMargStartIndex(std::ptrdiff_t index) {
  margStartIndex_ = index;
}

double TruncatedSvdSolver::getQRTolerance() const {
  if (factor_ && factor_->QRsym && factor_->QRnum)
    return factor_->tol;
  else
    return SPQR_DEFAULT_TOL;
}

double TruncatedSvdSolver::getSVDTolerance() const {
  return svdTolerance_;
}

const Eigen::VectorXd& TruncatedSvdSolver::getSingularValues() const {
  return singularValues_;
}

const Eigen::MatrixXd& TruncatedSvdSolver::getMatrixU() const {
  return matrixU_;
}

const Eigen::MatrixXd& TruncatedSvdSolver::getMatrixV() const {
  return matrixV_;
}

Eigen::MatrixXd TruncatedSvdSolver::getNullSpace() const {
  if (svdRank_ == -1 || svdRank_ > matrixV_.cols()) {
    return Eigen::MatrixXd(0, 0);
  }
  return matrixV_.rightCols(matrixV_.cols() - svdRank_);
}

Eigen::MatrixXd TruncatedSvdSolver::getRowSpace() const {
  if (svdRank_ == -1 || svdRank_ > matrixV_.cols()) {
    return Eigen::MatrixXd(0, 0);
  }
  return matrixV_.leftCols(svdRank_);
}

Eigen::MatrixXd TruncatedSvdSolver::getCovariance() const {
  if (svdRank_ == -1 || svdRank_ > matrixV_.cols()) {
    return Eigen::MatrixXd(0, 0);
  }
  return matrixV_.leftCols(svdRank_) *
    singularValues_.head(svdRank_).asDiagonal().inverse() *
    matrixV_.leftCols(svdRank_).adjoint();
}

Eigen::MatrixXd TruncatedSvdSolver::getRowSpaceCovariance() const {
  if (svdRank_ == -1) {
    return Eigen::MatrixXd(0, 0);
  }
  return singularValues_.head(svdRank_).asDiagonal().inverse();
}

double TruncatedSvdSolver::getSingularValuesLog2Sum() const {
  if (svdRank_ == -1) {
    return 0.0;
  }
  return singularValues_.head(svdRank_).array().log().sum() / std::log(2);
}

size_t TruncatedSvdSolver::getPeakMemoryUsage() const {
  return cholmod_.memory_usage;
}

size_t TruncatedSvdSolver::getMemoryUsage() const {
  return cholmod_.memory_inuse;
}

double TruncatedSvdSolver::getNumFlops() const {
  return cholmod_.SPQR_xstat[0];
}

double TruncatedSvdSolver::getLinearSolverTime() const {
  return linearSolverTime_;
}

double TruncatedSvdSolver::getMarginalAnalysisTime() const {
  return marginalAnalysisTime_;
}

double TruncatedSvdSolver::getSymbolicFactorizationTime() const {
  if (!factor_ || !factor_->QRsym) {
    return 0.0;
  }
  return cholmod_.other1[1];
}

double TruncatedSvdSolver::getNumericFactorizationTime() const {
  if (!factor_ || !factor_->QRnum) {
    return 0.0;
  }
  return cholmod_.other1[2];
}

void TruncatedSvdSolver::solve(cholmod_sparse* A, cholmod_dense* b,
                               std::ptrdiff_t j, Eigen::VectorXd& x) {
  CHECK_EQ(A->nrow, b->nrow);
  const bool hasQrPart = j > 0;
  const bool hasSvdPart = j < A->ncol;

  const double t0 = Timestamp::now();

  SelfFreeingCholmodPtr<cholmod_dense> G_l(nullptr, cholmod_);
  if(hasQrPart) {
    SelfFreeingCholmodPtr<cholmod_sparse> A_l(nullptr, cholmod_);
    A_l = columnSubmatrix(A, 0, j - 1, &cholmod_);

    if (options_.columnScaling) {
      G_l = columnScalingMatrix(A_l, &cholmod_, options_.epsNorm);
      bool success = cholmod_l_scale(G_l, CHOLMOD_COL, A_l, &cholmod_);
      CHECK(success) << "cholmod_l_scale failed.";
    }

    // Clear the cached symbolic QR-factorization if the matrix has changed.
    if (factor_ && factor_->QRsym &&
        (factor_->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
        factor_->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol) ||
        factor_->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax))) {
      clear();
    }

    // Calculate the symbolic factorization (if cache is not filled).
    if (!factor_) {
      const double t2 = Timestamp::now();
      factor_ = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
        SPQR_DEFAULT_TOL, A_l, &cholmod_);
      CHECK(factor_ != nullptr) << "SuiteSparseQR_symbolic failed.";

      const double t3 = Timestamp::now();
      cholmod_.other1[1] = t3 - t2;
    }

    const double qrTolerance = (options_.qrTol != -1.0) ? options_.qrTol :
      qrTol(A_l, &cholmod_, options_.epsQR);
    const double t2 = Timestamp::now();
    const bool status = SuiteSparseQR_numeric<double>(qrTolerance, A_l,
      factor_, &cholmod_);
    CHECK(status) << "SuiteSparseQR_numeric failed.";
    const double t3 = Timestamp::now();
    cholmod_.other1[2] = t3 - t2;
  } else {
    clear();
    cholmod_.other1[1] = cholmod_.other1[2] = 0.0;
  }

  Eigen::VectorXd x_r;
  SelfFreeingCholmodPtr<cholmod_dense> x_l(nullptr, cholmod_);
  SelfFreeingCholmodPtr<cholmod_dense> G_r(nullptr, cholmod_);
  if (hasSvdPart) {
    SelfFreeingCholmodPtr<cholmod_sparse> A_r(nullptr, cholmod_);
    A_r = columnSubmatrix(A, j, A->ncol - 1, &cholmod_);

    if (options_.columnScaling) {
      G_r = columnScalingMatrix(A_r, &cholmod_, options_.epsNorm);
      const bool success = cholmod_l_scale(G_r, CHOLMOD_COL, A_r, &cholmod_);
      CHECK(success) << "cholmod_l_scale failed.";
    }

    SelfFreeingCholmodPtr<cholmod_dense> b_r(nullptr, cholmod_);
    {
      SelfFreeingCholmodPtr<cholmod_sparse> A_rt(
          cholmod_l_transpose(A_r, 1, &cholmod_), cholmod_);
      CHECK(A_rt != nullptr) << "cholmod_l_transpose failed.";

      SelfFreeingCholmodPtr<cholmod_sparse> A_rtQ(nullptr, cholmod_);
      SelfFreeingCholmodPtr<cholmod_sparse> Omega(nullptr, cholmod_);
      reduceLeftHandSide(factor_, A_rt, &Omega, &A_rtQ, &cholmod_);
      analyzeSVD(Omega);
      b_r = reduceRightHandSide(factor_, A_rt, A_rtQ, b, &cholmod_);
    }
    solveSVD(b_r, singularValues_, matrixU_, matrixV_, svdRank_, x_r);
    if (hasQrPart) {
      x_l = solveQR(factor_, b, A_r, x_r, &cholmod_);
    }
  } else {
    analyzeSVD(nullptr);
    x_r.resize(0);
    if (hasQrPart) {
      x_l = solveQR(factor_, b, nullptr, x_r, &cholmod_);
    }
  }

  if (options_.columnScaling) {
    if(G_l){
      Eigen::Map<const Eigen::VectorXd> G_lEigen(
          reinterpret_cast<const double*>(G_l->x), G_l->nrow);
      Eigen::Map<Eigen::VectorXd> x_lEigen(
          reinterpret_cast<double*>(x_l->x), x_l->nrow);
      x_lEigen = G_lEigen.cwiseProduct(x_lEigen);
      G_l.reset(nullptr);
    }
    if(G_r){
      Eigen::Map<const Eigen::VectorXd> G_rEigen(
          reinterpret_cast<const double*>(G_r->x), G_r->nrow);
      x_r = G_rEigen.array() * x_r.array();
      G_r.reset(nullptr);
    }
  }

  x.resize(A->ncol);
  if(hasQrPart){
    Eigen::Map<Eigen::VectorXd> x_lEigen(
        reinterpret_cast<double*>(x_l->x), x_l->nrow);
    x.head(x_lEigen.size()) = x_lEigen;
  }
  if(hasSvdPart){
    x.tail(x_r.size()) = x_r;
  }

  linearSolverTime_ = Timestamp::now() - t0;
}

void TruncatedSvdSolver::analyzeSVD(cholmod_sparse * Omega) {
  if (Omega) {
    truncated_svd_solver::analyzeSVD(Omega, singularValues_, matrixU_,
                                     matrixV_);
    svdTolerance_ = (options_.svdTol != -1.0) ? options_.svdTol :
        rankTol(singularValues_, options_.epsSVD);
    svdRank_ = estimateNumericalRank(singularValues_, svdTolerance_);
    svdRankDeficiency_ = singularValues_.size() - svdRank_;
    svGap_ = svGap(singularValues_, svdRank_);
  } else {
    clearSvdAnalysisResultMembers();
    svdRank_ = 0;
    svdRankDeficiency_ = 0;
  }
}

void TruncatedSvdSolver::analyzeMarginal(cholmod_sparse* A, std::ptrdiff_t j) {
  const double t0 = Timestamp::now();
  const bool hasQrPart = j > 0;
  const bool hasSvdPart = j < A->ncol;

  if (hasQrPart) {
    SelfFreeingCholmodPtr<cholmod_sparse> A_l(
        columnSubmatrix(A, 0, j - 1, &cholmod_), cholmod_);
    if (factor_ && factor_->QRsym
        && (factor_->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow)
            || factor_->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol)
            || factor_->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax)))
      clear();
    if (!factor_) {
      const double t2 = Timestamp::now();
      factor_ = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
                                               SPQR_DEFAULT_TOL, A_l,
                                               &cholmod_);
      CHECK(factor_ != nullptr) << "SuiteSparseQR_symbolic failed.";

      const double t3 = Timestamp::now();
      cholmod_.other1[1] = t3 - t2;
    }

    const double t2 = Timestamp::now();
    const double qrTolerance =
        (options_.qrTol != -1.0) ? options_.qrTol :
            qrTol(A_l, &cholmod_, options_.epsQR);
    const bool success = SuiteSparseQR_numeric<double>(
        qrTolerance, A_l, factor_, &cholmod_);
    CHECK(success) << "SuiteSparseQR_numeric failed.";
    cholmod_.other1[2] = Timestamp::now() - t2;
  } else {
    clear();
    cholmod_.other1[1] = cholmod_.other1[2] = 0.0;
  }

  if (hasSvdPart) {
    SelfFreeingCholmodPtr<cholmod_sparse> A_r(
        columnSubmatrix(A, j, A->ncol - 1, &cholmod_), cholmod_);
    SelfFreeingCholmodPtr<cholmod_sparse> A_rt(
        cholmod_l_transpose(A_r, 1, &cholmod_), cholmod_);
    CHECK(A_rt != nullptr) << "cholmod_l_transpose failed.";

    SelfFreeingCholmodPtr<cholmod_sparse> Omega(nullptr, cholmod_);
    SelfFreeingCholmodPtr<cholmod_sparse> A_rtQ(nullptr, cholmod_);
    reduceLeftHandSide(factor_, A_rt, &Omega, &A_rtQ, &cholmod_);
    analyzeSVD(Omega);
  } else {
    analyzeSVD(nullptr);
  }

  marginalAnalysisTime_ = Timestamp::now() - t0;
}

void TruncatedSvdSolver::clearSvdAnalysisResultMembers() {
  svdRank_ = -1;
  svGap_ = std::numeric_limits<double>::infinity();
  svdRankDeficiency_ = -1;
  svdTolerance_ = -1.0;
  singularValues_.resize(0);
  matrixU_.resize(0, 0);
  matrixV_.resize(0, 0);
}

}  // namespace truncated_svd_solver
