#ifndef CAMERAFRAME_HPP
#define CAMERAFRAME_HPP

#include <opencv2/opencv.hpp> 

struct CameraFrame
{
    CameraFrame(const cv::Mat& aFrame, std::chrono::steady_clock::time_point aTimestamp) : 
        mFrame(aFrame), mTimestamp(aTimestamp) {}
    CameraFrame() {}

    cv::Mat mFrame; 
    std::chrono::steady_clock::time_point mTimestamp; 
    
    // TODO: populate with global pose of sensor at frame time
    //cv::Matx44f mT_g_c; 
};

#endif
