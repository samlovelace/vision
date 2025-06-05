
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
            throw std::invalid_argument("Cannot use monocular depth estimation without depth estimating neural network"); 
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
            // HACK until Detection is better defined with std::vector<cv::Mat> mFrame
            std::vector<cv::Mat> frames; 
            frames.push_back(detection.mFrame);  
            cv::Mat depthMap; 

            if(!mDepthEstimator->estimateDepth(frames, depthMap))
            {
                LOGW << "COuld not compute depth map!"; 
                continue; 
            }

            // computePose();
            
            mDepthMapVisQueue->push(depthMap); 
        }
    }
}

