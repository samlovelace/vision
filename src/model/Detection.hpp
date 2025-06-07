#ifndef DETECTION_HPP   
#define DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "CameraFrame.hpp"
#include <map>  

// TODO: eventually this has to hold the stereo image pair
struct Detection
{
    CameraFrame mFrame;
    std::map<std::string, std::vector<cv::Rect>> mDetectionsMap; 
};

#endif
