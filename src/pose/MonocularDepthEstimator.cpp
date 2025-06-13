
#include "MonocularDepthEstimator.h"
#include "plog/Log.h"

MonocularDepthEstimator::MonocularDepthEstimator(std::shared_ptr<InferenceHandler> anInferenceHandler) : 
    mInferenceHandler(anInferenceHandler)
{

}

MonocularDepthEstimator::~MonocularDepthEstimator()
{

}

bool MonocularDepthEstimator::estimateDepth(CameraOutput& aCameraOutput, cv::Mat& aDepthMapOut)
{
    // only monocular so take first image of frame pair 
    cv::Mat monoFrame = aCameraOutput.left.mFrame; 

    auto output = mInferenceHandler->runInference("depth", monoFrame); 

    if(nullptr == output)
    {
        return false; 
    }

    // safe to cast and assign output
    auto depthOutput = std::dynamic_pointer_cast<DepthOutput>(output);
    aDepthMapOut = depthOutput->depthMap; 
    return true; 
}
