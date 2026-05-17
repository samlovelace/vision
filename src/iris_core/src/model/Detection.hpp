#ifndef DETECTION_HPP   
#define DETECTION_HPP

#include <opencv2/opencv.hpp>
#include "CameraData.hpp"
#include "IModelOutput.hpp" 


struct Detection
{
    StampedCameraOutput mCameraOutput;
    std::shared_ptr<DetectionOutput> mDetections; 
};

#endif
