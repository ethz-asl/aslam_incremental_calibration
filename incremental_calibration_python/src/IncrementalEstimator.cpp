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

#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/calibration/core/IncrementalEstimator.h>

#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/SparseQRLinearSolverOptions.h>

void exportIncrementalEstimator() {
  using namespace boost::python;
  using namespace aslam::backend;
  using aslam::calibration::IncrementalEstimator;

  /// Export options for the IncrementalEstimator class
  class_<IncrementalEstimator::Options>("IncrementalEstimatorOptions", init<>())
    .def_readwrite("miTol", &IncrementalEstimator::Options::_miTol)
    .def_readwrite("verbose", &IncrementalEstimator::Options::_verbose)
    .def_readwrite("normTol", &IncrementalEstimator::Options::_normTol)
    .def_readwrite("epsTolSVD", &IncrementalEstimator::Options::_epsTolSVD)
    ;

  /// Export return value for the IncrementalEstimator class
  class_<IncrementalEstimator::ReturnValue>("IncrementalEstimatorReturnValue",
    init<>())
    .def_readwrite("batchAccepted",
      &IncrementalEstimator::ReturnValue::_batchAccepted)
    .def_readwrite("mi", &IncrementalEstimator::ReturnValue::_mi)
    .def_readwrite("rank", &IncrementalEstimator::ReturnValue::_rank)
    .def_readwrite("qrTol", &IncrementalEstimator::ReturnValue::_qrTol)
    .def_readwrite("numIterations",
      &IncrementalEstimator::ReturnValue::_numIterations)
    .def_readwrite("JStart", &IncrementalEstimator::ReturnValue::_JStart)
    .def_readwrite("JFinal", &IncrementalEstimator::ReturnValue::_JFinal)
    .def_readwrite("elapsedTime",
      &IncrementalEstimator::ReturnValue::_elapsedTime)
    .def_readwrite("cholmodMemoryUsage",
      &IncrementalEstimator::ReturnValue::_cholmodMemoryUsage)
    .def_readwrite("NS", &IncrementalEstimator::ReturnValue::_NS)
    .def_readwrite("CS", &IncrementalEstimator::ReturnValue::_CS)
    .def_readwrite("Sigma", &IncrementalEstimator::ReturnValue::_Sigma)
    .def_readwrite("SigmaP", &IncrementalEstimator::ReturnValue::_SigmaP)
    .def_readwrite("Omega", &IncrementalEstimator::ReturnValue::_Omega)
    ;

  /// Functions for querying the options
  IncrementalEstimator::Options& (IncrementalEstimator::*getOptions)() = 
    &IncrementalEstimator::getOptions;
  Optimizer2Options& (IncrementalEstimator::*getOptimizerOptions)() =
    &IncrementalEstimator::getOptimizerOptions;
  SparseQRLinearSolverOptions& (IncrementalEstimator::*getLinearSolverOptions)()
    = &IncrementalEstimator::getLinearSolverOptions;

  /// Removes a measurement batch from the estimator
  void (IncrementalEstimator::*removeBatch1)(size_t) =
    &IncrementalEstimator::removeBatch;
  void (IncrementalEstimator::*removeBatch2)(
    const IncrementalEstimator::BatchSP&) = &IncrementalEstimator::removeBatch;

  /// IncrementalEstimator class python exports
  class_<IncrementalEstimator, boost::shared_ptr<IncrementalEstimator>,
    boost::noncopyable>("IncrementalEstimator", init<size_t,
    const IncrementalEstimator::Options&, const SparseQRLinearSolverOptions&,
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
    .def("getMutualInformation", &IncrementalEstimator::getMutualInformation)
    .def("getMargGroupId", &IncrementalEstimator::getMargGroupId)
    .def("getJacobianTranspose", &IncrementalEstimator::getJacobianTranspose,
      return_internal_reference<>())
    .def("getRank", &IncrementalEstimator::getRank)
    .def("getRankDeficiency", &IncrementalEstimator::getRankDeficiency)
    .def("getMarginalRank", &IncrementalEstimator::getMarginalRank)
    .def("getMarginalRankDeficiency",
      &IncrementalEstimator::getMarginalRankDeficiency)
    .def("getQRTol", &IncrementalEstimator::getQRTol)
    .def("getCholmodMemoryUsage", &IncrementalEstimator::getCholmodMemoryUsage)
    .def("getMarginalizedCovariance",
      &IncrementalEstimator::getMarginalizedCovariance,
      return_internal_reference<>())
    .def("getProjectedMarginalizedCovariance",
      &IncrementalEstimator::getProjectedMarginalizedCovariance,
      return_internal_reference<>())
    .def("getMarginalizedNullSpace",
      &IncrementalEstimator::getMarginalizedNullSpace,
      return_internal_reference<>())
    .def("getMarginalizedColumnSpace",
      &IncrementalEstimator::getMarginalizedColumnSpace,
      return_internal_reference<>())
    .def("getMarginalizedInformationMatrix",
      &IncrementalEstimator::getMarginalizedInformationMatrix,
      return_internal_reference<>())
    ;
}
