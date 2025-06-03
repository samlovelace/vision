#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H
 
#include <yaml-cpp/yaml.h>
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
#include "IDepthEstimator.hpp"

class PoseEstimationHandler 
{ 
public:
    PoseEstimationHandler(const YAML::Node& aPoseEstConfig, std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue);
    ~PoseEstimationHandler();

    void run(); 

private:
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 

    std::shared_ptr<IDepthEstimator> mDepthEstimator; 

};
#endif //POSEESTIMATOR_H