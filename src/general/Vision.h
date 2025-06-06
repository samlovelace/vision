#ifndef VISION_H
#define VISION_H
 
#include "CameraHandler.h"
#include "ConcurrentQueue.hpp" 
#include "ObjectDetectionHandler.h"
#include "PoseEstimationHandler.h"
#include "InferenceHandler.h"
#include "IModule.hpp"
#include "CameraFrame.hpp"

#include <thread> 

class Vision : public IModule
{ 
public:
    Vision();
    ~Vision();

    void start() override; 
    void stop(); 

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 

    std::vector<std::thread> mThreads; 

    std::shared_ptr<CameraHandler> mCameraHandler; 
    std::shared_ptr<ConcurrentQueue<CameraFrame>> mFrameQueue; 

    std::shared_ptr<ObjectDetectionHandler> mDetector; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> m2DVisQueue; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mDepthFrameVisQueue; 

    std::shared_ptr<PoseEstimationHandler> mPoseEstimationHandler;

    bool mVisualize; 
   
};
#endif //VISION_H