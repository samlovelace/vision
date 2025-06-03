
#include "PoseEstimationHandler.h"
#include "DepthEstimatorFactory.h"

PoseEstimationHandler::PoseEstimationHandler(const YAML::Node& aPoseEstConfig, std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue) : 
    mDetectionQueue(aDetectionQueue)
{
    std::string depthEstType = aPoseEstConfig["depth_estimation"].as<std::string>(); 
    mDepthEstimator = DepthEstimatorFactory::create(depthEstType);
    
    if(nullptr == mDepthEstimator)
    {
        throw std::invalid_argument("Invalid pose estimation configuration"); 
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
            // compute pose of object detected
            mDepthEstimator->estimateDepth();
            // computePose(); 
        }
    }
}

