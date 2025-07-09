
#include "StereoDepthEstimator.h"

StereoDepthEstimator::StereoDepthEstimator()
{

}

StereoDepthEstimator::~StereoDepthEstimator()
{

}

bool StereoDepthEstimator::estimateDepth(CameraOutput& aFramePair, cv::Mat& aDepthMap)
{
    cv::Mat leftGray, rightGray; 

    cv::cvtColor(aFramePair.left.mFrame, leftGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(aFramePair.right.mFrame, rightGray, cv::COLOR_BGR2GRAY);

    auto stereo = cv::StereoSGBM::create(0, 16*5, 2);

    cv::Mat disparity;
    stereo->compute(leftGray, rightGray, disparity);

    cv::Mat disparity_float;
    disparity.convertTo(disparity_float, CV_32F, 1.0/16.0);

    double minVal, maxVal;
    cv::minMaxLoc(disparity_float, &minVal, &maxVal);
    std::cout << "Disparity range: " << minVal << " to " << maxVal << std::endl;

    aDepthMap = disparity_float; 

    return true; 
}

