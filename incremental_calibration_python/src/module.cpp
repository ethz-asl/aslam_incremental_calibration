/******************************************************************************
 * Copyright (C) 2013 by Paul Furgale and Jerome Maye                         *
 * jerome.maye@gmail.com                                                      *
 ******************************************************************************/

/** \file module.cpp
    \brief This file defines the top-level python exports.
  */

#include <numpy_eigen/boost_python_headers.hpp>

//void exportVisionDataAssociation();
void exportOptimizationProblem();
void exportIncrementalEstimator();
void exportLinearSolver();

BOOST_PYTHON_MODULE(libincremental_calibration_python) {
  exportOptimizationProblem();
  exportIncrementalEstimator();
  exportLinearSolver();
}
