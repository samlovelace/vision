
#include "PoseEstimationHandler.h"
#include "MonocularDepthEstimator.h"


PoseEstimationHandler::PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                                             std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                                             std::shared_ptr<InferenceHandler> anInferenceHandler, 
                                             std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue) : 
    mDetectionQueue(aDetectionQueue), mDepthMapVisQueue(aDepthMapVisQueue)
{
    std::string depthEstType = aPoseEstConfig["depth_estimation"].as<std::string>(); 
    //mDepthEstimator = DepthEstimatorFactory::create(depthEstType);

    if("monocular" == depthEstType)
    {
        if(!anInferenceHandler->hasModelType("depth"))
        {
            throw std::runtime_error("Cannot use monocular depth estimation without depth estimating neural network"); 
        }
        mDepthEstimator = std::make_shared<MonocularDepthEstimator>(anInferenceHandler); 
    }
    else
    {
        LOGE << "Unsupported depth estimator of type: " << depthEstType; 
        mDepthEstimator = nullptr; 
    }
    
    if(nullptr == mDepthEstimator)
    {
        throw std::runtime_error("Invalid pose estimation configuration"); 
    }
}

PoseEstimationHandler::~PoseEstimationHandler()
{

}

void PoseEstimationHandler::run()
{
    while(true)
    {
        Detection detection; 
        if(mDetectionQueue->pop(detection))
        {
            cv::Mat depthMap; 

            if(!mDepthEstimator->estimateDepth(detection.mCameraOutput.frames, depthMap))
            {
                LOGW << "Could not compute depth map!"; 
                continue; 
            }

            cv::resize(depthMap, depthMap, cv::Size(640, 480)); 

            // for(auto& [obj, detections] : detection.mDetections->mDetections)
            // {
            //     for(auto& det : detections)
            //     {
            //         det.bounding_box
            //     }
            // }
            

            // computePose();
            mDepthMapVisQueue->push(depthMap); 
        }
    }
}

