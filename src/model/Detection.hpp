#ifndef DETECTION_HPP   
#define DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "CameraData.hpp"
#include <map>  


struct Detection
{
    StampedCameraOutput mCameraOutput;
    std::map<std::string, std::vector<cv::Rect>> mDetectionsMap; 
};

#endif
