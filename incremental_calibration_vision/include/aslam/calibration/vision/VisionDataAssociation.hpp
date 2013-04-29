#ifndef _VISIONDATAASSOCIATION_H_
#define _VISIONDATAASSOCIATION_H_

#include <aslam/cameras.hpp>
#include <aslam/ImageSynchronizer.hpp>
#include <aslam/Match.hpp>
#include <aslam/NullUndistorter.hpp>
#include <aslam/Undistorter.hpp>
#include <aslam/SurfFrameBuilder.hpp>
#include <sm/eigen/traits.hpp>
#include <aslam/DenseMatcher.hpp>
#include <aslam/DescriptorTrackingAlgorithm.hpp>
#include <sm/PropertyTree.hpp>

namespace aslam {
    namespace calibration {
        
        class VisionDataAssociation
        {
        public:
            typedef aslam::cameras::DistortedPinholeCameraGeometry camera_t;
            typedef aslam::cameras::NullUndistorter<camera_t> single_undistorter_t;
            typedef aslam::Undistorter2<single_undistorter_t,single_undistorter_t> undistorter_t;
            typedef aslam::ImageSynchronizer<undistorter_t> synchronizer_t;
            typedef aslam::CameraSystem<undistorter_t::camera_system_definition_t> camera_system_t;

            

            VisionDataAssociation(const sm::kinematics::Transformation & T_v_cl,
                                  boost::shared_ptr<camera_t> leftCamera,
                                  const sm::kinematics::Transformation & T_v_cr,
                                  boost::shared_ptr<camera_t> rightCamera,
                                  double descriptorDistanceThreshold,
                                  double disparityTrackingThreshold,
                                  double disparityKeyframeThreshold,
                                  int numTracksThreshold);

            VisionDataAssociation(const sm::PropertyTree & config);
                        
            virtual ~VisionDataAssociation();
            
            void addImage(const aslam::Time & stamp,
                          int cameraIndex,
                          const cv::Mat & image);
            
            
            void reset();
            
        private:
            double computeDisparity( const boost::shared_ptr<MultiFrame> & F0,
                                     const boost::shared_ptr<MultiFrame> & F1,
                                     const KeypointIdentifierMatch & match);
                                   

            MultiFrameId _nextFrameId;
            LandmarkId _nextLandmarkId;

            boost::shared_ptr<MultiFrame> _previousFrame;

            boost::shared_ptr<synchronizer_t> _synchronizer;
            std::map< MultiFrameId, boost::shared_ptr<MultiFrame> > _frames;
            std::vector< KeypointIdentifierMatch > _matches;
            aslam::DenseMatcher _matcher;
            aslam::DescriptorTrackingAlgorithm _tracking;
            double _disparityKeyframeThreshold;
            int _numTracksThreshold;
        };

    } // namespace calibration
} // namespace aslam



#endif /* _VISIONDATAASSOCIATION_H_ */
