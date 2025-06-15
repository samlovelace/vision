#ifndef CAMERAFRAME_HPP
#define CAMERAFRAME_HPP

#include <opencv2/opencv.hpp> 
#include <memory>
#include "CameraParams.hpp"

struct CameraFrame
{
    CameraFrame(const cv::Mat& aFrame, std::chrono::system_clock::time_point aTimestamp) : 
        mFrame(aFrame), mTimestamp(aTimestamp) {}
    CameraFrame() {}

    cv::Mat mFrame; 
    std::chrono::system_clock::time_point mTimestamp; 
};

struct CameraOutput
{
    CameraFrame left; 
    CameraFrame right; 
    bool isStereo = false; 
};

struct StampedCameraOutput
{
    CameraOutput frames; 
    std::shared_ptr<CameraParams> mParams; 
    cv::Matx44f T_G_C; 
};

#endif
