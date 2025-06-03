#ifndef MODELHANDLER_H
#define MODELHANDLER_H
 
#include <yaml-cpp/yaml.h>
#include "ModelContext.hpp"
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"

#include "InferenceHandler.h"
 
class ModelHandler 
{ 
public:
    ModelHandler(const YAML::Node& aModelsConfig, 
                 std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                 std::shared_ptr<InferenceHandler> anInferenceHandler);

    ~ModelHandler();

    void run();

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 

    std::shared_ptr<ConcurrentQueue<cv::Mat>> mFrameQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mVisQueue; 

    void renderDetections(Detection& aDetection);
   
};
#endif //MODELHANDLER_H