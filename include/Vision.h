#ifndef VISION_H
#define VISION_H
 
#include "CameraHandler.h"
#include "ConcurrentQueue.hpp" 
#include "ModelHandler.h"

#include <thread> 

class Vision 
{ 
public:
    Vision();
    ~Vision();

    void start(); 
    void stop(); 

private:

    std::vector<std::thread> mThreads; 

    std::shared_ptr<CameraHandler> mCameraHandler; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mFrameQueue; 

    std::shared_ptr<ModelHandler> mDetector; 
    //std::shared_ptr<ConcurrentQueue<Detections> mDetectionQueue; 

    //std::shared_ptr<PoseEstimator> mPoseEstimator
   
};
#endif //VISION_H