// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

void exportVisionDataAssociation()
{
    using namespace boost::python;
    using namespace aslam::calibration;

    class_<VisionDataAssociation>("VisionDataAssociation", 
                                  init<const sm::kinematics::Transformation &,
                                       boost::shared_ptr<camera_t>,
                                       const sm::kinematics::Transformation &,
                                       boost::shared_ptr<camera_t> ,
                                       double,
                                       double,
                                       double,
                                       int>("VisionDataAssociation(const sm::kinematics::Transformation & T_v_cl, boost::shared_ptr<camera_t> leftCamera, const sm::kinematics::Transformation & T_v_cr,boost::shared_ptr<camera_t> rightCamera, double descriptorDistanceThreshold, double disparityTrackingThreshold, double disparityKeyframeThreshold, int numTracksThreshold)")
                                  .def(init<const sm::PropertyTree &>())
                                  ;
    
}
