#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H
 
#include <yaml-cpp/yaml.h>
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
#include "IDepthEstimator.hpp"
#include "InferenceHandler.h"

class PoseEstimationHandler 
{ 
public:
    PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                          std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                          std::shared_ptr<InferenceHandler> anInferenceHandler, 
                          std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue); 
    ~PoseEstimationHandler();

    void run(); 

private:
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mDepthMapVisQueue; 
    std::shared_ptr<IDepthEstimator> mDepthEstimator; 

};
#endif //POSEESTIMATOR_H