#ifndef _VISIONDATAASSOCIATION_H_
#define _VISIONDATAASSOCIATION_H_

#include <aslam/cameras.hpp>
#include <aslam/ImageSynchronizer.hpp>
#include <aslam/Match.hpp>

namespace aslam {
    namespace calibration {
        
        class VisionDataAssociation
        {
        public:
            typedef aslam::cameras::DistortedPinholeCameraGeometry camera_t;

            //VisionDataAssociation();
            VisionDataAssociation(boost::shared_ptr<camera_t> leftCamera, 
                                  boost::shared_ptr<camera_t> rightCamera);
                        
            virtual ~VisionDataAssociation();
            
            void addImage(const aslam::Time & stamp,
                          int cameraIndex,
                          const cv::Mat & image);
            
        private:
            boost::shared_ptr<ImageSynchronizerBase> _synchronizer;
            std::map< MultiFrameId, boost::shared_ptr<MultiFrame> > _frames;
            std::vector< KeypointIdentifierMatch > _matches;
        };

    } // namespace calibration
} // namespace aslam



#endif /* _VISIONDATAASSOCIATION_H_ */
