#include <aslam/calibration/vision/VisionDataAssociation.hpp>


namespace aslam {
    namespace calibration {
        
        VisionDataAssociation::VisionDataAssociation(const sm::kinematics::Transformation & T_v_cl,
                                                     boost::shared_ptr<camera_t> leftCamera,
                                                     const sm::kinematics::Transformation & T_v_cr,
                                                     boost::shared_ptr<camera_t> rightCamera)
        {
            boost::shared_ptr<single_undistorter_t> u1(new single_undistorter_t(leftCamera, aslam::cameras::interpolation::Linear, 1.0, false));
            boost::shared_ptr<single_undistorter_t> u2(new single_undistorter_t(rightCamera, aslam::cameras::interpolation::Linear, 1.0, false));
            boost::shared_ptr<undistorter_t> u( new undistorter_t(u1,u2) );
            
            boost::shared_ptr<FrameBuilder> sfb(new SurfFrameBuilder(100));
            
            boost::shared_ptr< camera_system_t > cs(new camera_system_t( T_v_cl,
                                                                         leftCamera,
                                                                         T_v_cr,
                                                                         rightCamera) );

            _synchronizer.reset( new synchronizer_t( cs,
                                                     u,
                                                     sfb,
                                                     0.001, // double timestampTolerance,
                                                     true,  // bool doBackProjection,
                                                     false  // bool doBackProjectionUncertainty
                                     )
                );
            
            _nextId = MultiFrameId(0);
        }

       
                        
        VisionDataAssociation::~VisionDataAssociation()
        {

        }
            
        void VisionDataAssociation::addImage(const aslam::Time & stamp,
                                             int cameraIndex,
                                             const cv::Mat & image)
        {
            boost::shared_ptr<MultiFrame> frame = _synchronizer->addImage(stamp,cameraIndex,image);
            if(frame)
            {
                frame->setId(_nextId++);
                _frames[frame->id()] = frame;
                
                // For each camera, run a disparity-based tracking.
                

            }
        }
            
        
        
    } // namespace calibration
} // namespace aslam
