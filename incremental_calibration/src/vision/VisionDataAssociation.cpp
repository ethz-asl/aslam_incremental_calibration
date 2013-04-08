#include <aslam/calibration/vision/VisionDataAssociation.hpp>
#include <sm/logging.hpp>


namespace aslam {
    namespace calibration {
        
        VisionDataAssociation::VisionDataAssociation(const sm::kinematics::Transformation & T_v_cl,
                                                     boost::shared_ptr<camera_t> leftCamera,
                                                     const sm::kinematics::Transformation & T_v_cr,
                                                     boost::shared_ptr<camera_t> rightCamera,
                                                     double descriptorDistanceThreshold,
                                                     double disparityTrackingThreshold,
                                                     double disparityKeyframeThreshold,
                                                     int numTracksThreshold) :
            _disparityKeyframeThreshold(disparityKeyframeThreshold),
            _numTracksThreshold(numTracksThreshold)
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
            
            _nextFrameId = MultiFrameId(0);
            _nextLandmarkId = LandmarkId(0);
            _tracking.setParameters(descriptorDistanceThreshold, disparityTrackingThreshold);

        }


        VisionDataAssociation::VisionDataAssociation(const sm::PropertyTree & config) :
            _tracking( sm::PropertyTree( config, "descriptorTracking" ) )
        {
            _synchronizer.reset( new synchronizer_t( sm::PropertyTree( config, "synchronizer" ) ) );
            _nextFrameId = MultiFrameId(0);
            _nextLandmarkId = LandmarkId(0);
        }


       
                        
        VisionDataAssociation::~VisionDataAssociation()
        {

        }
            
        void VisionDataAssociation::addImage(const aslam::Time & stamp,
                                             int cameraIndex,
                                             const cv::Mat & image)
        {
            boost::shared_ptr<MultiFrame> frame = _synchronizer->addImage(stamp,cameraIndex,image);
            
            std::vector< KeypointIdentifierMatch > f2fMatches;
            if(frame && _previousFrame)
            {
                                
                // For each camera, run a disparity-based tracking with the last camera.
                for(size_t i = 0; i < frame->numCameras(); ++i)
                {
                    _tracking.setFrames(_previousFrame->id(),
                                        i,
                                        _previousFrame->getFrame(i).get(), 
                                        _nextFrameId,
                                        i,
                                        frame->getFrame(i).get());

                    _matcher.match(_tracking);
                    
                    // Retrieve the matches
                    _tracking.insertMatches(f2fMatches);
                    SM_INFO_STREAM("Camera " << i << " had " << _tracking.getMatches().size() << " matches");
                }
                SM_INFO_STREAM("Total matches: " << f2fMatches.size());
                if((int)f2fMatches.size() > _numTracksThreshold)
                {
                    SM_INFO_STREAM("Track threshold passed! Checking disparities");
                    // get all disparities
                    std::vector<double> disparities;
                    disparities.reserve(f2fMatches.size());
                    for(size_t i = 0; i < f2fMatches.size(); ++i)
                    {
                        disparities.push_back( computeDisparity( _previousFrame, frame, f2fMatches[i] ) );
                    }
                
                    size_t middleIdx = disparities.size() / 2;
                    // compute the median disparity of all tracks.
                    std::vector<double>::iterator middle = disparities.begin() + middleIdx + 1;
                    partial_sort(disparities.begin(), middle, disparities.end()); 

                    double medianDisparity = disparities[middleIdx];
                    SM_INFO_STREAM("Median disparity: " << medianDisparity);
                    // if the median disparity is above a threshold, add the image and tracks.
                    if(medianDisparity > _disparityKeyframeThreshold)
                    {
                        SM_INFO_STREAM("Meadian disparity threshold passed! Saving the frame");
                        frame->setId(_nextFrameId);
                        _nextFrameId++;
                        _frames[frame->id()] = frame;
                        _previousFrame = frame;
                        _matches.insert(_matches.end(), f2fMatches.begin(), f2fMatches.end());
                    }
                    
                }
                else
                {
                    _previousFrame = frame;
                }
            }
        }
            
        double VisionDataAssociation::computeDisparity( const boost::shared_ptr<MultiFrame> & F0,
                                                        const boost::shared_ptr<MultiFrame> & F1,
                                                        const KeypointIdentifierMatch & match)
        {

            KeypointBase & K0 = F0->keypoint(match.index[0]);
            KeypointBase & K1 = F1->keypoint(match.index[1]);

            if(K0.landmarkId().isSet())
            {
                K1.setLandmarkId(K0.landmarkId());
            }
            else
            {
                LandmarkId id = _nextLandmarkId++;
                K0.setLandmarkId(id);
                K1.setLandmarkId(id);
            }
            return ( K0.vsMeasurement() - K1.vsMeasurement() ).norm();
            
        }

        void VisionDataAssociation::reset()
        {
            // std::map< MultiFrameId, boost::shared_ptr<MultiFrame> > _frames;
            _frames.clear();
            // std::vector< KeypointIdentifierMatch > _matches;
            _matches.clear();
            // std::map< LandmarkId, std::vector< KeypointIdentifier > > _observations;
            //_observations.clear();
            // sm::eigen::MapTraits< LandmarkId, Eigen::Vector4d >::map_t _landmarks;
            //_landmarks.clear();

        }
        
        
        
    } // namespace calibration
} // namespace aslam
