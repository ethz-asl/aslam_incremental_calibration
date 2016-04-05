#include "truncated-svd-solver/tsvd-solver.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <iostream>

#include <SuiteSparseQR.hpp>

#include "truncated-svd-solver/linear-algebra-helpers.h"
#include "truncated-svd-solver/timing.h"

namespace truncated_svd_solver {

void deleteCholdmodPtr(cholmod_sparse * & ptr, cholmod_common & cholmod){
  if(ptr) cholmod_l_free_sparse(&ptr, &cholmod);
}
void deleteCholdmodPtr(cholmod_dense * & ptr, cholmod_common & cholmod){
  if(ptr) cholmod_l_free_dense(&ptr, &cholmod);
}

template <typename T>
struct SelfFreeingCholmodPtr {
  explicit SelfFreeingCholmodPtr(T * ptr, cholmod_common & cholmod) : _ptr(ptr), _cholmod(cholmod) { }
  ~SelfFreeingCholmodPtr(){
    reset(NULL);
  }
  void reset(T * ptr){
    deleteCholdmodPtr(_ptr, _cholmod);
    _ptr = ptr;
  }
  SelfFreeingCholmodPtr & operator = (T * ptr){
    reset(ptr);
    return *this;
  }

  operator T * () {
    return _ptr;
  }
  T * operator -> () {
    return _ptr;
  }

  T ** operator & () {
    return &_ptr;
  }
private:
  cholmod_common & _cholmod;
  T * _ptr;
};

TruncatedSvdSolver::TruncatedSvdSolver(const Options& options) :
    _options(options),
    _factor(NULL),
    _svdRank(-1),
    _svGap(std::numeric_limits<double>::infinity()),
    _svdRankDeficiency(-1),
    _margStartIndex(-1),
    _svdTolerance(-1.0),
    _linearSolverTime(0.0),
    _marginalAnalysisTime(0.0) {
  cholmod_l_start(&_cholmod);
  _cholmod.SPQR_grain = 16;  // Maybe useless.
}

TruncatedSvdSolver::LinearSolver(const sm::PropertyTree& config) :
    _factor(NULL),
    _svdRank(-1),
    _svGap(std::numeric_limits<double>::infinity()),
    _svdRankDeficiency(-1),
    _margStartIndex(-1),
    _svdTolerance(-1.0),
    _linearSolverTime(0.0),
    _marginalAnalysisTime(0.0) {
  cholmod_l_start(&_cholmod);
  _cholmod.SPQR_grain = 16; // maybe useless
  _options.columnScaling = config.getBool("columnScaling",
    _options.columnScaling);
  _options.epsNorm = config.getDouble("epsNorm", _options.epsNorm);
  _options.epsSVD = config.getDouble("epsSVD", _options.epsSVD);
  _options.epsQR = config.getDouble("epsQR", _options.epsQR);
  _options.svdTol = config.getDouble("svdTol", _options.svdTol);
  _options.qrTol = config.getDouble("qrTol", _options.qrTol);
  _options.verbose = config.getBool("verbose", _options.verbose);
  // TODO(schneith): ideally, use constructor delegation, not available for now.
}

TruncatedSvdSolver::~TruncatedSvdSolver() {
  clear();
  cholmod_l_finish(&_cholmod);
  if (_options.verbose && getMemoryUsage())
    std::cerr << __PRETTY_FUNCTION__ << ": cholmod memory leak"
      << std::endl;
}

const TruncatedSvdSolver::Options& TruncatedSvdSolver::getOptions() const {
  return _options;
}

TruncatedSvdSolver::Options& TruncatedSvdSolver::getOptions() {
  return _options;
}

std::string TruncatedSvdSolver::name() const {
  return std::string("marginal_spqr_svd");
}

std::ptrdiff_t TruncatedSvdSolver::getSVDRank() const {
  return _svdRank;
}

std::ptrdiff_t TruncatedSvdSolver::getSVDRankDeficiency() const {
  return _svdRankDeficiency;
}

double TruncatedSvdSolver::getSvGap() const {
  return _svGap;
}

std::ptrdiff_t TruncatedSvdSolver::getQRRank() const {
  if (_factor && _factor->QRnum)
    return _factor->rank;
  else
    return 0;
}

std::ptrdiff_t TruncatedSvdSolver::getQRRankDeficiency() const {
  if (_factor && _factor->QRsym && _factor->QRnum)
    return _factor->QRsym->n - _factor->rank;
  else
    return 0;
}

std::ptrdiff_t TruncatedSvdSolver::getMargStartIndex() const {
  return _margStartIndex;
}

void TruncatedSvdSolver::setMargStartIndex(std::ptrdiff_t index) {
  _margStartIndex = index;
}

double TruncatedSvdSolver::getQRTolerance() const {
  if (_factor && _factor->QRsym && _factor->QRnum)
    return _factor->tol;
  else
    return SPQR_DEFAULT_TOL;
}

double TruncatedSvdSolver::getSVDTolerance() const {
  return _svdTolerance;
}

const Eigen::VectorXd& TruncatedSvdSolver::getSingularValues() const {
  return _singularValues;
}

const Eigen::MatrixXd& TruncatedSvdSolver::getMatrixU() const {
  return _matrixU;
}

const Eigen::MatrixXd& TruncatedSvdSolver::getMatrixV() const {
  return _matrixV;
}

Eigen::MatrixXd TruncatedSvdSolver::getNullSpace() const {
  if (_svdRank != -1 && _svdRank <= _matrixV.cols())
    return _matrixV.rightCols(_matrixV.cols() - _svdRank);
  else
    return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd TruncatedSvdSolver::getRowSpace() const {
  if (_svdRank != -1 && _svdRank <= _matrixV.cols())
    return _matrixV.leftCols(_svdRank);
  else
    return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd TruncatedSvdSolver::getCovariance() const {
  if (_svdRank != -1 && _svdRank <= _matrixV.cols())
    return _matrixV.leftCols(_svdRank) *
      _singularValues.head(_svdRank).asDiagonal().inverse() *
      _matrixV.leftCols(_svdRank).adjoint();
  else
    return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd TruncatedSvdSolver::getRowSpaceCovariance() const {
  if (_svdRank != -1)
    return _singularValues.head(_svdRank).asDiagonal().inverse();
  else
    return Eigen::MatrixXd(0, 0);
}

double TruncatedSvdSolver::getSingularValuesLog2Sum() const {
  if (_svdRank != -1)
    return _singularValues.head(_svdRank).array().log().sum() / std::log(2);
  else
    return 0.0;
}

size_t TruncatedSvdSolver::getPeakMemoryUsage() const {
  return _cholmod.memory_usage;
}

size_t TruncatedSvdSolver::getMemoryUsage() const {
  return _cholmod.memory_inuse;
}

double TruncatedSvdSolver::getNumFlops() const {
  return _cholmod.SPQR_xstat[0];
}

double TruncatedSvdSolver::getLinearSolverTime() const {
  return _linearSolverTime;
}

double TruncatedSvdSolver::getMarginalAnalysisTime() const {
  return _marginalAnalysisTime;
}

double TruncatedSvdSolver::getSymbolicFactorizationTime() const {
  if (_factor && _factor->QRsym)
    return _cholmod.other1[1];
  else
    return 0.0;
}

double TruncatedSvdSolver::getNumericFactorizationTime() const {
  if (_factor && _factor->QRnum)
    return _cholmod.other1[2];
  else
    return 0.0;
}

double TruncatedSvdSolver::rhsJtJrhs() {
  return 0.0;
}

void TruncatedSvdSolver::solve(cholmod_sparse* A, cholmod_dense* b,
    std::ptrdiff_t j, Eigen::VectorXd& x) {
  const double t0 = Timestamp::now();
  if (A->nrow != b->nrow)
    throw InvalidOperationException("inconsistent A and b", __FILE__,
      __LINE__, __PRETTY_FUNCTION__);

  const bool hasQrPart = j > 0;
  const bool hasSvdPart = j < A->ncol;

  SelfFreeingCholmodPtr<cholmod_dense> G_l(NULL, _cholmod);
  if(hasQrPart) {
    SelfFreeingCholmodPtr<cholmod_sparse> A_l(NULL, _cholmod);
    A_l = columnSubmatrix(A, 0, j - 1, &_cholmod);
    if (_options.columnScaling) {
      G_l = columnScalingMatrix(A_l, &_cholmod, _options.epsNorm);
      if (!cholmod_l_scale(G_l, CHOLMOD_COL, A_l, &_cholmod)) {
        throw InvalidOperationException("cholmod_l_scale failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
    }
    if (_factor && _factor->QRsym &&
        (_factor->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
        _factor->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol) ||
        _factor->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax)))
      clear();
    if (!_factor) {
      const double t2 = Timestamp::now();
      _factor = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
        SPQR_DEFAULT_TOL, A_l, &_cholmod);
      const double t3 = Timestamp::now();
      _cholmod.other1[1] = t3 - t2;
      if (_factor == NULL) {
        throw InvalidOperationException("SuiteSparseQR_symbolic failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      }
    }
    const double qrTolerance = (_options.qrTol != -1.0) ? _options.qrTol :
      qrTol(A_l, &_cholmod, _options.epsQR);
    const double t2 = Timestamp::now();
    const int status = SuiteSparseQR_numeric<double>(qrTolerance, A_l,
      _factor, &_cholmod);
    const double t3 = Timestamp::now();
    _cholmod.other1[2] = t3 - t2;
    if (!status) {
      throw InvalidOperationException("SuiteSparseQR_numeric failed",
        __FILE__, __LINE__, __PRETTY_FUNCTION__);
    }
  } else {
    clear();
    _cholmod.other1[1] = _cholmod.other1[2] = 0.0;
  }

  Eigen::VectorXd x_r;
  SelfFreeingCholmodPtr<cholmod_dense> x_l(NULL, _cholmod);
  SelfFreeingCholmodPtr<cholmod_dense> G_r(NULL, _cholmod);
  if(hasSvdPart){
    SelfFreeingCholmodPtr<cholmod_sparse> A_r(NULL, _cholmod);
    A_r = columnSubmatrix(A, j, A->ncol - 1, &_cholmod);

    if (_options.columnScaling) {
      G_r = columnScalingMatrix(A_r, &_cholmod, _options.epsNorm);
      if (!cholmod_l_scale(G_r, CHOLMOD_COL, A_r, &_cholmod)) {
        throw InvalidOperationException("cholmod_l_scale failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
    }

    SelfFreeingCholmodPtr<cholmod_dense> b_r(NULL, _cholmod);
    {
      SelfFreeingCholmodPtr<cholmod_sparse> A_rt(cholmod_l_transpose(A_r, 1, &_cholmod), _cholmod);
      if (A_rt == NULL) {
        throw InvalidOperationException("cholmod_l_transpose failed", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      }
      SelfFreeingCholmodPtr<cholmod_sparse> A_rtQ(NULL, _cholmod);
      SelfFreeingCholmodPtr<cholmod_sparse> Omega(NULL, _cholmod);
      reduceLeftHandSide(_factor, A_rt, &Omega, &A_rtQ, &_cholmod);
      analyzeSVD(Omega);
      b_r = reduceRightHandSide(_factor, A_rt, A_rtQ, b, &_cholmod);
    }
    solveSVD(b_r, _singularValues, _matrixU, _matrixV, _svdRank, x_r);
    if(hasQrPart){
      x_l = solveQR(_factor, b, A_r, x_r, &_cholmod);
    }
  } else {
    analyzeSVD(NULL);
    x_r.resize(0);
    if(hasQrPart){
      x_l = solveQR(_factor, b, NULL, x_r, &_cholmod);
    }
  }

  if (_options.columnScaling) {
    if(G_l){
      Eigen::Map<const Eigen::VectorXd> G_lEigen(reinterpret_cast<const double*>(G_l->x), G_l->nrow);
      Eigen::Map<Eigen::VectorXd> x_lEigen(reinterpret_cast<double*>(x_l->x), x_l->nrow);
      x_lEigen = G_lEigen.cwiseProduct(x_lEigen);
      G_l.reset(NULL);
    }
    if(G_r){
      Eigen::Map<const Eigen::VectorXd> G_rEigen(reinterpret_cast<const double*>(G_r->x), G_r->nrow);
      x_r = G_rEigen.array() * x_r.array();
      G_r.reset(NULL);
    }
  }

  x.resize(A->ncol);
  if(hasQrPart){
    Eigen::Map<Eigen::VectorXd> x_lEigen(reinterpret_cast<double*>(x_l->x), x_l->nrow);
    x.head(x_lEigen.size()) = x_lEigen;
  }
  if(hasSvdPart){
    x.tail(x_r.size()) = x_r;
  }

  _linearSolverTime = Timestamp::now() - t0;
}

void TruncatedSvdSolver::analyzeSVD(cholmod_sparse * Omega) {
  if(Omega){
    calibration::analyzeSVD(Omega, _singularValues, _matrixU, _matrixV);
    _svdTolerance = (_options.svdTol != -1.0) ? _options.svdTol : rankTol(_singularValues, _options.epsSVD);
    _svdRank = estimateNumericalRank(_singularValues, _svdTolerance);
    _svdRankDeficiency = _singularValues.size() - _svdRank;
    _svGap = svGap(_singularValues, _svdRank);
  } else {
    clearSvdAnalysisResultMembers();
    _svdRank = 0;
    _svdRankDeficiency = 0;
  }
}

void TruncatedSvdSolver::analyzeMarginal(cholmod_sparse* A, std::ptrdiff_t j) {
  const double t0 = Timestamp::now();

  const bool hasQrPart = j > 0;
  const bool hasSvdPart = j < A->ncol;

  if(hasQrPart){
    SelfFreeingCholmodPtr<cholmod_sparse> A_l(columnSubmatrix(A, 0, j - 1, &_cholmod), _cholmod);
    if (_factor && _factor->QRsym &&
        (_factor->QRsym->m != static_cast<std::ptrdiff_t>(A_l->nrow) ||
        _factor->QRsym->n != static_cast<std::ptrdiff_t>(A_l->ncol) ||
        _factor->QRsym->anz != static_cast<std::ptrdiff_t>(A_l->nzmax)))
      clear();
    if (!_factor) {
      const double t2 = Timestamp::now();
      _factor = SuiteSparseQR_symbolic<double>(SPQR_ORDERING_BEST,
        SPQR_DEFAULT_TOL, A_l, &_cholmod);
      const double t3 = Timestamp::now();
      _cholmod.other1[1] = t3 - t2;
      if (_factor == NULL) {
        throw InvalidOperationException("SuiteSparseQR_symbolic failed",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
      }
    }
    const double qrTolerance = (_options.qrTol != -1.0) ? _options.qrTol :
      qrTol(A_l, &_cholmod, _options.epsQR);
    const double t2 = Timestamp::now();
    const int status = SuiteSparseQR_numeric<double>(qrTolerance, A_l,
      _factor, &_cholmod);
    _cholmod.other1[2] = Timestamp::now() - t2;
    if (!status)
      throw InvalidOperationException("SuiteSparseQR_numeric failed",
        __FILE__, __LINE__, __PRETTY_FUNCTION__);
  }else{
    clear();
    _cholmod.other1[1] = _cholmod.other1[2] = 0.0;
  }
  if(hasSvdPart){
    SelfFreeingCholmodPtr<cholmod_sparse> A_r(columnSubmatrix(A, j, A->ncol - 1, &_cholmod), _cholmod);
    SelfFreeingCholmodPtr<cholmod_sparse> A_rt(cholmod_l_transpose(A_r, 1, &_cholmod), _cholmod);
    if (A_rt == NULL) {
      throw InvalidOperationException("cholmod_l_transpose failed", __FILE__,
        __LINE__, __PRETTY_FUNCTION__);
    }
    SelfFreeingCholmodPtr<cholmod_sparse> Omega(NULL, _cholmod);
    SelfFreeingCholmodPtr<cholmod_sparse> A_rtQ(NULL, _cholmod);
    reduceLeftHandSide(_factor, A_rt, &Omega, &A_rtQ, &_cholmod);
    analyzeSVD(Omega);
  } else {
    analyzeSVD(NULL);
  }

  _marginalAnalysisTime = Timestamp::now() - t0;
}

void TruncatedSvdSolver::clearSvdAnalysisResultMembers() {
  _svdRank = -1;
  _svGap = std::numeric_limits<double>::infinity();
  _svdRankDeficiency = -1;
  _svdTolerance = -1.0;
  _singularValues.resize(0);
  _matrixU.resize(0, 0);
  _matrixV.resize(0, 0);
}

void TruncatedSvdSolver::clear() {
  if (_factor) {
    SuiteSparseQR_free<double>(&_factor, &_cholmod);
    _factor = NULL;
  }
  clearSvdAnalysisResultMembers();
  _linearSolverTime = 0.0;
  _marginalAnalysisTime = 0.0;
}

}  // namespace truncated_svd_solver
