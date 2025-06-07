#ifndef CAMERAFRAME_HPP
#define CAMERAFRAME_HPP

#include <opencv2/opencv.hpp> 

struct CameraFrame
{
    CameraFrame(const cv::Mat& aFrame) : mFrame(aFrame) {}
    CameraFrame() {}
    
    cv::Mat mFrame; 
    
    // TODO: populate with global pose of sensor at frame time
    //cv::Matx44f mT_g_c; 
};

#endif
