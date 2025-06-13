#ifndef IDEPTHESTIMATOR_HPP
#define IDEPTHESTIMATOR_HPP
 
#include <vector> 
#include <opencv2/opencv.hpp>
#include "CameraData.hpp"
 
class IDepthEstimator 
{ 
public:
    ~IDepthEstimator() = default; 

    virtual bool estimateDepth(CameraOutput& aCameraOutput, cv::Mat& aDepthMapOut) = 0; 

private:
   
};
#endif //IDEPTHESTIMATOR_HPP