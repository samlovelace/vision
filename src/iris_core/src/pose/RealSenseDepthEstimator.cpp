
#include "RealSenseDepthEstimator.h"
#include "plog/Log.h"

RealSenseDepthEstimator::RealSenseDepthEstimator()
{

}

RealSenseDepthEstimator::~RealSenseDepthEstimator()
{

}

bool RealSenseDepthEstimator::estimateDepth(CameraOutput& aCameraOutput, cv::Mat& aDepthFrame)
{
    if(aCameraOutput.depth.mFrame.empty())
    {
        LOGE << "Depth frame empty"; 
        return false; 
    }

    aDepthFrame = aCameraOutput.depth.mFrame.clone(); 
    return true; 
}
