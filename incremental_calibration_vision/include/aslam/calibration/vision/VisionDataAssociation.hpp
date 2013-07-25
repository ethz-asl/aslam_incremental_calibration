#ifndef _VISIONDATAASSOCIATION_H_
#define _VISIONDATAASSOCIATION_H_

#include <aslam/cameras.hpp>
#include <aslam/ImageSynchronizer.hpp>
#include <aslam/Match.hpp>
#include <aslam/NCameraPipeline.hpp>
#include <sm/eigen/traits.hpp>
#include <aslam/DenseMatcher.hpp>
#include <aslam/DescriptorTrackingAlgorithm.hpp>
#include <aslam/EpipolarMatchingAlgorithm.hpp>
#include <sm/PropertyTree.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>


namespace aslam {
    namespace calibration {
        
        class VisionDataAssociation
        {
        public:
            typedef boost::shared_ptr<aslam::calibration::OptimizationProblem> OptimizationProblemSP;
            typedef aslam::backend::DesignVariable DesignVariable;
            
            /// \brief Initialize the data association algorithm with a property tree
            VisionDataAssociation(const sm::PropertyTree & config);
                        
            virtual ~VisionDataAssociation();
            
            /// \brief Add an image to the pipeline.
            void addImage(const aslam::Time & stamp,
                          int cameraIndex,
                          const cv::Mat & image);
                        
            /// \brief Add the contents of the internal state to the optimization problem
            ///        using the bspline pose representation passed in.
            void addToProblem( aslam::splines::BSplinePoseDesignVariable & T_w_vk, OptimizationProblemSP problem );

            /// \brief reset the internal state (do this in between batches)
            void reset();

            /// \brief how many calibration design variables does this class have?
            size_t numCalibrationDesignVariables();
            
            /// \brief get calibration design variable i
            boost::shared_ptr< DesignVariable > getCalibrationDesignVariable( size_t i );
            
            
        private:
            double computeDisparity( const boost::shared_ptr<MultiFrame> & F0,
                                     const boost::shared_ptr<MultiFrame> & F1,
                                     const KeypointIdentifierMatch & match);

            void doOfovMatching( boost::shared_ptr<MultiFrame> mf );
                                   
            
            /// \brief The pipeline for synchronising images and getting features
            boost::shared_ptr< NCameraPipeline > _pipeline;

            /// \brief The next available multiframe id
            MultiFrameId _nextFrameId;

            /// \brief The next available landmark id
            LandmarkId _nextLandmarkId;

            /// \brief the previous frame used.
            boost::shared_ptr<MultiFrame> _previousFrame;

            std::map< MultiFrameId, boost::shared_ptr<MultiFrame> > _frames;
            std::vector< KeypointIdentifierMatch > _matches;
            aslam::DenseMatcher _matcher;
            aslam::DescriptorTrackingAlgorithm _tracking;
            aslam::EpipolarMatchingAlgorithm _ofovMatching;
            double _disparityKeyframeThreshold;
            int _numTracksThreshold;

            std::vector< boost::shared_ptr<DesignVariable> > _designVariables;
        };

    } // namespace calibration
} // namespace aslam



#endif /* _VISIONDATAASSOCIATION_H_ */
