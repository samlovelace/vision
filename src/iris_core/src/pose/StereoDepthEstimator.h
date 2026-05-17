#ifndef STEREODEPTHESTIMATOR_H
#define STEREODEPTHESTIMATOR_H
 
#include "IDepthEstimator.hpp"
 
class StereoDepthEstimator : public IDepthEstimator
{ 
public:
    StereoDepthEstimator();
    ~StereoDepthEstimator();

    bool estimateDepth(CameraOutput& aFrames, cv::Mat& aDepthMapOut) override; 

private:
   
};
#endif //STEREODEPTHESTIMATOR_H