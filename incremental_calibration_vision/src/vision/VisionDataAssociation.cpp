#include <aslam/calibration/vision/VisionDataAssociation.hpp>
#include <sm/logging.hpp>


namespace aslam {
    namespace calibration {
        

        VisionDataAssociation::VisionDataAssociation(const sm::PropertyTree & config) :
            _tracking( sm::PropertyTree( config, "descriptorTracking" ) ),
            _ofovMatching( sm::PropertyTree( config, "ofovMatching" ) )
        {
            std::string pname = config.getString("pipelineName");
            SM_INFO_STREAM("Using the pipeline named " << pname);
            _pipeline.reset( new NCameraPipeline( sm::PropertyTree( config, pname) ) );

            _nextFrameId = MultiFrameId(0);
            _nextLandmarkId = LandmarkId(0);

            // \todo create the calibration design variables.
            
            
        }


       
                        
        VisionDataAssociation::~VisionDataAssociation()
        {

        }
            
        void VisionDataAssociation::addImage(const aslam::Time & stamp,
                                             int cameraIndex,
                                             const cv::Mat & image)
        {
            boost::shared_ptr<MultiFrame> frame = _pipeline->addImage(stamp,cameraIndex,image);

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
                        
                        doOfovMatching( frame );
                        
                    }
                    
                }
                else
                {
                    // This is a failure recovery. We weren't able to track
                    // enough features so throw out the previous frame
                    // and start again.
                    _previousFrame = frame;
                }
            }
            
            // This happens only once after a reset...
            if( frame && !_previousFrame ) {
                _previousFrame = frame;
            }

        }
            
        double VisionDataAssociation::computeDisparity( const boost::shared_ptr<MultiFrame> & F0,
                                                        const boost::shared_ptr<MultiFrame> & F1,
                                                        const KeypointIdentifierMatch & match)
        {
            KeypointBase & K0 = F0->keypoint(match.index[0]);
            KeypointBase & K1 = F1->keypoint(match.index[1]);

            return ( K0.vsMeasurement() - K1.vsMeasurement() ).norm();
            
        }

        void VisionDataAssociation::reset()
        {
            // std::map< MultiFrameId, boost::shared_ptr<MultiFrame> > _frames;
            _frames.clear();
            // std::vector< KeypointIdentifierMatch > _matches;
            _matches.clear();
            
            _previousFrame.reset();

        }


        /// \brief Add the contents of the internal state to the optimization problem
        ///        using the bspline pose representation passed in.
        void VisionDataAssociation::addToProblem( aslam::splines::BSplinePoseDesignVariable & T_w_vk, OptimizationProblemSP problem ) {

            // This is where the magic happens.
            
            // \todo: initialize a list of landmark design variables.
            

            // For each match..
            for(size_t i = 0; i < _matches.size(); ++i) {
                //KeypointIdentifierMatch & match = _matches[i];
                
                // Check the MF0 keypoint for a landmark id
                // if it does not have one
                //    triangulate the point
                //    Check the reprojection error for bad triangulation
                //    if the triangulation was good
                //       Create a new landmark
                //       Add the new landmark to the problem
                //       Set the landmark id in MF 0
                //       Create a reprojection error for MF0
                //       Add the new RE it to the problem
                // endif
                
                // if the last step was successful
                //    Set the landmark id in MF 1
                //    Create a reprojection error for MF1
                //    Add it to the problem
                // endif
                
                
            }

        }
        
        /// \brief how many calibration design variables does this class have?
        size_t VisionDataAssociation::numCalibrationDesignVariables() {
            return _designVariables.size();
        }
            
            /// \brief get calibration design variable i
        boost::shared_ptr< aslam::backend::DesignVariable > VisionDataAssociation::getCalibrationDesignVariable( size_t i ) {
            SM_ASSERT_LT( std::runtime_error, i, _designVariables.size(), "Index out of bounds");
            return _designVariables[i];
        }
        

        void VisionDataAssociation::doOfovMatching( boost::shared_ptr<MultiFrame> mf ) {

            for(size_t i = 0; i < mf->numFrames(); ++i) {
                for( size_t j = i + 1; j < mf->numFrames(); ++j ) {
                    
                    // void setMatchData(MultiFrameId fidA,
                    //                   int cameraIndexA,
                    //                   FrameBase * cameraA,
                    //                   MultiFrameId fidB,
                    //                   int cameraIndexB,
                    //                   FrameBase * cameraB,
                    //                   const cameras::ImageMask * overlapAB,
                    //                   const cameras::ImageMask * overlapBA,
                    //                   const sm::kinematics::Transformation & T_A_B);

                    _ofovMatching.setMatchData(mf->id(),
                                           i,
                                           mf->getFrame(i).get(), 
                                           mf->id(),
                                           j,
                                           mf->getFrame(j).get(),
                                           NULL,
                                           NULL,
                                           mf->T_ci_cj(i,j)
                        );
                    
                    _matcher.match(_ofovMatching);
                    std::vector< KeypointIdentifierMatch > ofovMatches;
                    _ofovMatching.swapMatches(ofovMatches);
                    _matches.insert(_matches.end(), ofovMatches.begin(), ofovMatches.end());                    

                }
            }

        }
        
    } // namespace calibration
} // namespace aslam
