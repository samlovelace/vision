#ifndef REALSENSEDEPTHESTIMATOR_H
#define REALSENSEDEPTHESTIMATOR_H
 
#include "IDepthEstimator.hpp" 

class RealSenseDepthEstimator : public IDepthEstimator
{ 
public:
    RealSenseDepthEstimator();
    ~RealSenseDepthEstimator();

    bool estimateDepth(CameraOutput& aCameraOutput, cv::Mat& aDepthMapOut) override;  

private:
   
};
#endif //REALSENSEDEPTHESTIMATOR_H