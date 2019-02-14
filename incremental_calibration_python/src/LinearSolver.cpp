/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file LinearSolver.cpp
    \brief This file defines the python exports for the LinearSolver class.
  */

#include <aslam-tsvd-solver/aslam-tsvd-solver.h>
#include <boost/shared_ptr.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/PropertyTree.hpp>

using namespace boost::python;
using namespace aslam::backend;
using namespace sm;

void exportLinearSolver() {
  typedef AslamTruncatedSvdSolver LinearSolver;
  typedef AslamTruncatedSvdSolver::Options LinearSolverOptions;

  /// Export LinearSolverOptions structure
  class_<LinearSolverOptions>("LinearSolverOptions", init<>())
    .def_readwrite("columnScaling", &LinearSolverOptions::columnScaling)
    .def_readwrite("epsNorm", &LinearSolverOptions::epsNorm)
    .def_readwrite("epsSVD", &LinearSolverOptions::epsSVD)
    .def_readwrite("epsQR", &LinearSolverOptions::epsQR)
    .def_readwrite("svdTol", &LinearSolverOptions::svdTol)
    .def_readwrite("qrTol", &LinearSolverOptions::qrTol)
    .def_readwrite("verbose", &LinearSolverOptions::verbose)
    ;

  /// Function for querying the options
  LinearSolverOptions& (LinearSolver::*getOptions)() =
    &LinearSolver::getOptions;

  /// Export LinearSolver class
  class_<LinearSolver, boost::shared_ptr<LinearSolver>,
    bases<LinearSystemSolver>, boost::noncopyable>("LinearSolver",
    init<const LinearSolverOptions&>())
    .def(init<const PropertyTree&>())
    .def("getOptions", getOptions, return_internal_reference<>())
    ;
}
