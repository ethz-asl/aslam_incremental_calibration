/******************************************************************************
 * Copyright (C) 2013 by Paul Furgale and Jerome Maye                         *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file OptimizationProblem.cpp
    \brief This file defines the python exports for the OptimizationProblem
           class.
  */

#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <aslam/backend/DesignVariable.hpp>

void exportOptimizationProblem() {
  using namespace boost::python;
  using aslam::calibration::OptimizationProblem;

  /// Python export for OptimizationProblem class
  class_<aslam::calibration::OptimizationProblem,
    boost::shared_ptr<aslam::calibration::OptimizationProblem>,
    boost::noncopyable,
    boost::python::bases<aslam::backend::OptimizationProblemBase> >(
      "CalibrationOptimizationProblem", init<>())
    .def("addDesignVariable", &OptimizationProblem::addDesignVariable)
    .def("addErrorTerm", &OptimizationProblem::addErrorTerm)
    .def("clear", &OptimizationProblem::clear)
    ;

  using aslam::calibration::IncrementalOptimizationProblem;
  class_< IncrementalOptimizationProblem::DesignVariablesP>(
      "DesignVariablesVector")
    .def(vector_indexing_suite<
      IncrementalOptimizationProblem::DesignVariablesP>());

  /// Remove an optimization problem
  void (IncrementalOptimizationProblem::*remove1)(size_t idx) =
    &IncrementalOptimizationProblem::remove;

  aslam::calibration::OptimizationProblem*
    (IncrementalOptimizationProblem::*getOp)(size_t) =
    &IncrementalOptimizationProblem::getOptimizationProblem;

  /// Python export for IncrementalOptimizationProblem class
  class_<aslam::calibration::IncrementalOptimizationProblem,
    boost::shared_ptr<aslam::calibration::IncrementalOptimizationProblem>,
    boost::noncopyable,
    boost::python::bases<aslam::backend::OptimizationProblemBase> >(
      "IncrementalOptimizationProblem", init<>())
    .def("add", &IncrementalOptimizationProblem::add)
    .def("remove", remove1)
    .def("clear", &IncrementalOptimizationProblem::clear)
    .def("getNumOptimizationProblems",
      &IncrementalOptimizationProblem::getNumOptimizationProblems)
    .def("getOptimizationProblem",
      getOp, boost::python::return_internal_reference<>())
    ;

}
