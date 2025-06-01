#ifndef DETECTION_HPP   
#define DETECTION_HPP

#include <opencv2/opencv.hpp>
#include <vector> 

// TODO: eventually this has to hold the stereo image pair
struct Detection
{
    cv::Mat mFrame;
    std::vector<cv::Rect> mDetections; 
};

#endif
