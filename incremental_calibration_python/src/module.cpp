// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>


void exportVisionDataAssociation();
void exportOptimizationProblem();
void exportIncrementalEstimator();

// The title of this library must match exactly
BOOST_PYTHON_MODULE(libincremental_calibration_python)
{
    // fill this in with boost::python export code
    exportVisionDataAssociation();
    exportOptimizationProblem();
    exportIncrementalEstimator();

}
