#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/calibration/core/IncrementalOptimizationProblem.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <aslam/backend/DesignVariable.hpp>


void exportOptimizationProblem()
{

    using namespace boost::python;
    using aslam::calibration::OptimizationProblem;
    class_<aslam::calibration::OptimizationProblem,  // the type being wrapped
           boost::shared_ptr<aslam::calibration::OptimizationProblem>, // tell boost that we support shared pointers
           boost::noncopyable, // tell boost that we will never pass or return this by value.
           boost::python::bases<aslam::backend::OptimizationProblemBase> 
           >("CalibrationOptimizationProblem", init<>())
        .def("addDesignVariable", &OptimizationProblem::addDesignVariable)
        .def("addErrorTerm", &OptimizationProblem::addErrorTerm)
        .def("clear", &OptimizationProblem::clear)
    // \todo add more functions
        //.def("", &OptimizationProblem::);
        //.def("", &OptimizationProblem::);
        //.def("", &OptimizationProblem::);
        //.def("", &OptimizationProblem::);
        ;

    using aslam::calibration::IncrementalOptimizationProblem;
    class_< IncrementalOptimizationProblem::DesignVariablesP >("DesignVariablesVector")
        .def(vector_indexing_suite<IncrementalOptimizationProblem::DesignVariablesP >());


      /// Remove an optimization problem
      void (IncrementalOptimizationProblem::*remove1)(size_t idx) = &IncrementalOptimizationProblem::remove;

aslam::calibration::OptimizationProblem * (IncrementalOptimizationProblem::*getOp)(size_t) = &IncrementalOptimizationProblem::getOptimizationProblem;

    class_<aslam::calibration::IncrementalOptimizationProblem,  // the type being wrapped
           boost::shared_ptr<aslam::calibration::IncrementalOptimizationProblem>, // tell boost that we support shared pointers
           boost::noncopyable, // tell boost that we will never pass or return this by value.
           boost::python::bases<aslam::backend::OptimizationProblemBase> 
           >("IncrementalOptimizationProblem", init<>())
        .def("add", &IncrementalOptimizationProblem::add)
        .def("remove", remove1)
        .def("clear", &IncrementalOptimizationProblem::clear)
        .def("getNumOptimizationProblems", &IncrementalOptimizationProblem::getNumOptimizationProblems)
        .def("getOptimizationProblem", getOp, boost::python::return_internal_reference<>())
        //.def("", &IncrementalOptimizationProblem::)
           ;



}
