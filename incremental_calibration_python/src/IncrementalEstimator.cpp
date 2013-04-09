#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>

void exportIncrementalEstimator()
{
    using namespace boost::python;
    using aslam::calibration::IncrementalEstimator;

    class_<IncrementalEstimator::Options>("IncrementalEstimatorOptions", init<>())
        .def_readwrite("miTol",&IncrementalEstimator::Options::_miTol)
        .def_readwrite("qrTol",&IncrementalEstimator::Options::_qrTol)
        .def_readwrite("verbose",&IncrementalEstimator::Options::_verbose)
        .def_readwrite("colNorm",&IncrementalEstimator::Options::_colNorm)
        ;

    IncrementalEstimator::Options & (IncrementalEstimator::*getOptions)() = &IncrementalEstimator::getOptions;


    class_<IncrementalEstimator, 
           boost::shared_ptr<IncrementalEstimator>, 
           boost::noncopyable
           >("IncrementalEstimator",init<const IncrementalEstimator::DVContainer &,
             const IncrementalEstimator::Options &>())
           .def(init<const IncrementalEstimator::DVContainer &>())
        .def("getOptions",getOptions,return_internal_reference<>())
           ;
}
