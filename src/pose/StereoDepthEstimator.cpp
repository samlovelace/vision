
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

    auto stereo = cv::StereoSGBM::create(
    0,            // minDisparity
    128,          // numDisparities
    5             // blockSize
    );
    // stereo->setP1(8*3*5*5);
    // stereo->setP2(32*3*5*5);
    // stereo->setUniquenessRatio(5);
    // stereo->setSpeckleWindowSize(50);
    // stereo->setSpeckleRange(2);


    cv::Mat disparity;
    stereo->compute(leftGray, rightGray, disparity);

    cv::imshow("raw disparity", disparity); 
    cv::waitKey(1); 

    cv::Mat disparity_float;
    disparity.convertTo(disparity_float, CV_32F, 1.0/16.0);

    double minVal, maxVal;
    cv::minMaxLoc(disparity_float, &minVal, &maxVal);
    std::cout << "Disparity range: " << minVal << " to " << maxVal << std::endl;

    aDepthMap = disparity_float; 

    return true; 
}

