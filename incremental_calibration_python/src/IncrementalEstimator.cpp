#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <sm/python/stl_converters.hpp>

boost::python::list getPermutationVector(const aslam::calibration::IncrementalEstimator * ie)
{
    boost::python::list out;
    std::vector<ssize_t> vec = ie->getPermutationVector();

    sm::python::stlToList(vec.begin(), vec.end(), out);

    return out;
}

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
           >("IncrementalEstimator",init<size_t,
             const IncrementalEstimator::Options &>("IncrementalEstimator(groupId, Options) -- The group id should identify the calibration parameters"))
           .def(init<size_t>("IncrementalEstimator(groupId) -- The group id should identify the calibration parameters"))
           .def("getOptions",getOptions,return_internal_reference<>())
           .def("addBatch", &IncrementalEstimator::addBatch)
           .def("numBatches", &IncrementalEstimator::numBatches)
           .def("removeBatch", &IncrementalEstimator::removeBatch)
           .def("getMarginalizedCovariance", &IncrementalEstimator::getMarginalizedCovariance )
           .def("getMutualInformation", &IncrementalEstimator::getMutualInformation)
           .def("getMargGroupId", &IncrementalEstimator::getMargGroupId)
        .def("getJacobianTranspose", &IncrementalEstimator::getJacobianTranspose, return_internal_reference<>())
           .def("getRank", &IncrementalEstimator::getRank)
           .def("getQRTol", &IncrementalEstimator::getQRTol)
        .def("getR", &IncrementalEstimator::getR,return_internal_reference<>())
           
           ;
}
