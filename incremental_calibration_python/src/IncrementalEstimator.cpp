/******************************************************************************
 * Copyright (C) 2013 by Paul Furgale and Jerome Maye                         *
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

/** \file IncrementalEstimator.cpp
    \brief This file defines the python exports for the IncrementalEstimator
           class.
  */

#include <boost/shared_ptr.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <aslam/backend/Optimizer2Options.hpp>

#include <aslam/calibration/core/LinearSolverOptions.h>
#include <aslam/calibration/core/IncrementalEstimator.h>

using namespace boost::python;
using namespace aslam::backend;
using namespace aslam::calibration;

/// This functions gets rid of the reference
Eigen::MatrixXd getMarginalizedNullSpace(const IncrementalEstimator* ie) {
  return ie->getMarginalizedNullSpace();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getScaledMarginalizedNullSpace(const IncrementalEstimator* ie) {
  return ie->getScaledMarginalizedNullSpace();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getMarginalizedColumnSpace(const IncrementalEstimator* ie) {
  return ie->getMarginalizedColumnSpace();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getMarginalizedCovariance(const IncrementalEstimator* ie) {
  return ie->getMarginalizedCovariance();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getProjectedMarginalizedCovariance(const IncrementalEstimator*
    ie) {
  return ie->getProjectedMarginalizedCovariance();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getSingularValues(const IncrementalEstimator* ie) {
  return ie->getSingularValues();
}

/// This functions gets rid of the reference
Eigen::MatrixXd getScaledSingularValues(const IncrementalEstimator* ie) {
  return ie->getScaledSingularValues();
}

void exportIncrementalEstimator() {
  /// Export options for the IncrementalEstimator class
  class_<IncrementalEstimator::Options>("IncrementalEstimatorOptions", init<>())
    .def_readwrite("miTol", &IncrementalEstimator::Options::miTol)
    .def_readwrite("verbose", &IncrementalEstimator::Options::verbose)
    ;

  /// Export return value for the IncrementalEstimator class
  class_<IncrementalEstimator::ReturnValue>("IncrementalEstimatorReturnValue",
    init<>())
    .def_readwrite("batchAccepted",
      &IncrementalEstimator::ReturnValue::batchAccepted)
    .def_readwrite("mutualInformation",
      &IncrementalEstimator::ReturnValue::mutualInformation)
    .def_readwrite("rank", &IncrementalEstimator::ReturnValue::rank)
    .def_readwrite("rankDeficiency",
      &IncrementalEstimator::ReturnValue::rankDeficiency)
    .def_readwrite("marginalRank",
      &IncrementalEstimator::ReturnValue::marginalRank)
    .def_readwrite("marginalRankDeficiency",
      &IncrementalEstimator::ReturnValue::marginalRankDeficiency)
    .def_readwrite("svdTolerance",
      &IncrementalEstimator::ReturnValue::svdTolerance)
    .def_readwrite("qrTolerance",
      &IncrementalEstimator::ReturnValue::qrTolerance)
    .def_readwrite("nullSpace", &IncrementalEstimator::ReturnValue::nullSpace)
    .def_readwrite("scaledNullSpace",
      &IncrementalEstimator::ReturnValue::scaledNullSpace)
    .def_readwrite("columnSpace",
      &IncrementalEstimator::ReturnValue::columnSpace)
    .def_readwrite("covariance", &IncrementalEstimator::ReturnValue::covariance)
    .def_readwrite("projectedCovariance",
      &IncrementalEstimator::ReturnValue::projectedCovariance)
    .def_readwrite("singularValues",
      &IncrementalEstimator::ReturnValue::singularValues)
    .def_readwrite("scaledSingularValues",
      &IncrementalEstimator::ReturnValue::scaledSingularValues)
    .def_readwrite("numIterations",
      &IncrementalEstimator::ReturnValue::numIterations)
    .def_readwrite("JStart", &IncrementalEstimator::ReturnValue::JStart)
    .def_readwrite("JFinal", &IncrementalEstimator::ReturnValue::JFinal)
    .def_readwrite("elapsedTime",
      &IncrementalEstimator::ReturnValue::elapsedTime)
    ;

  /// Functions for querying the options
  IncrementalEstimator::Options& (IncrementalEstimator::*getOptions)() = 
    &IncrementalEstimator::getOptions;
  Optimizer2Options& (IncrementalEstimator::*getOptimizerOptions)() =
    &IncrementalEstimator::getOptimizerOptions;
  LinearSolverOptions& (IncrementalEstimator::*getLinearSolverOptions)()
    = &IncrementalEstimator::getLinearSolverOptions;

  /// Removes a measurement batch from the estimator
  void (IncrementalEstimator::*removeBatch1)(size_t) =
    &IncrementalEstimator::removeBatch;
  void (IncrementalEstimator::*removeBatch2)(
    const IncrementalEstimator::BatchSP&) = &IncrementalEstimator::removeBatch;

  /// Export IncrementalEstimator class
  class_<IncrementalEstimator, boost::shared_ptr<IncrementalEstimator>,
    boost::noncopyable>("IncrementalEstimator", init<size_t,
    const IncrementalEstimator::Options&, const LinearSolverOptions&,
    const Optimizer2Options&>("IncrementalEstimator(groupId, Options, "
    "LinearSolverOptions, OptimizerOptions) -- The group id should identify "
    "the calibration parameters"))
    .def(init<size_t>("IncrementalEstimator(groupId) -- The group id should "
      "identify the calibration parameters"))
    .def("getOptions", getOptions, return_internal_reference<>())
    .def("getOptimizerOptions", getOptimizerOptions,
      return_internal_reference<>())
    .def("getLinearSolverOptions", getLinearSolverOptions,
      return_internal_reference<>())
    .def("addBatch", &IncrementalEstimator::addBatch)
    .def("reoptimize", &IncrementalEstimator::reoptimize)
    .def("getNumBatches", &IncrementalEstimator::getNumBatches)
    .def("removeBatch", removeBatch1)
    .def("removeBatch", removeBatch2)
    .def("getMargGroupId", &IncrementalEstimator::getMargGroupId)
    .def("getMutualInformation", &IncrementalEstimator::getMutualInformation)
    .def("getJacobianTranspose", &IncrementalEstimator::getJacobianTranspose,
      return_internal_reference<>())
    .def("getRank", &IncrementalEstimator::getRank)
    .def("getRankDeficiency", &IncrementalEstimator::getRankDeficiency)
    .def("getMarginalRank", &IncrementalEstimator::getMarginalRank)
    .def("getMarginalRankDeficiency",
      &IncrementalEstimator::getMarginalRankDeficiency)
    .def("getSVDTolerance", &IncrementalEstimator::getSVDTolerance)
    .def("getQRTolerance", &IncrementalEstimator::getQRTolerance)
    .def("getPeakMemoryUsage", &IncrementalEstimator::getPeakMemoryUsage)
    .def("getMemoryUsage", &IncrementalEstimator::getMemoryUsage)
    .def("getNumFlops", &IncrementalEstimator::getNumFlops)
    .def("getMarginalizedNullSpace", &getMarginalizedNullSpace)
    .def("getScaledMarginalizedNullSpace", &getScaledMarginalizedNullSpace)
    .def("getMarginalizedColumnSpace", &getMarginalizedColumnSpace)
    .def("getMarginalizedCovariance", &getMarginalizedCovariance)
    .def("getProjectedMarginalizedCovariance",
      &getProjectedMarginalizedCovariance)
    .def("getSingularValues", &getSingularValues)
    .def("getScaledSingularValues", &getScaledSingularValues)
    ;
}
