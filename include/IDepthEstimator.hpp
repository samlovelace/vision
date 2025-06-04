#ifndef IDEPTHESTIMATOR_HPP
#define IDEPTHESTIMATOR_HPP
 
#include <vector> 
#include <opencv2/opencv.hpp>
 
class IDepthEstimator 
{ 
public:
    ~IDepthEstimator() = default; 

    virtual bool estimateDepth(std::vector<cv::Mat>& aFramePair, cv::Mat& aDepthMapOut) = 0; 

private:
   
};
#endif //IDEPTHESTIMATOR_HPP