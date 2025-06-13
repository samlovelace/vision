#ifndef ObjectDetectionHandler_H
#define ObjectDetectionHandler_H
 
#include <yaml-cpp/yaml.h>
#include "ModelContext.hpp"
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
#include "CameraData.hpp"

#include "InferenceHandler.h"
 
class ObjectDetectionHandler 
{ 
public:
    ObjectDetectionHandler(const YAML::Node& aModelsConfig, 
                 std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueueVector, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                 std::shared_ptr<InferenceHandler> anInferenceHandler);

    ~ObjectDetectionHandler();

    void run();

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 

    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mVisQueue; 

    void renderDetections(Detection& aDetection);
   
};
#endif //ObjectDetectionHandler_H