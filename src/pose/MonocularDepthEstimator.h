#ifndef MONOCULARDEPTHESTIMATOR_H
#define MONOCULARDEPTHESTIMATOR_H
 
#include "IDepthEstimator.hpp"
#include "InferenceHandler.h"
 
class MonocularDepthEstimator : public IDepthEstimator
{ 
public:
    MonocularDepthEstimator(std::shared_ptr<InferenceHandler> anInferenceHandler);
    ~MonocularDepthEstimator();

    bool estimateDepth(CameraOutput& aCameraOutput, cv::Mat& aDepthMapOut) override; 

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 
   
};
#endif //MONOCULARDEPTHESTIMATOR_H