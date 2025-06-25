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
    ObjectDetectionHandler(const YAML::Node& aDetectionConfig, 
                 std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueueVector, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                 std::shared_ptr<InferenceHandler> anInferenceHandler);

    ~ObjectDetectionHandler();

    void run();
    void setObjectOfInterest(const std::string& anObjectType) {mObjectToLocate = anObjectType; } 

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mVisQueue; 

    float mMinConfidenceThreshold; 
    float mSimilarDetectionThreshold;
    std::string mObjectToLocate; 

    void renderDetections(Detection& aDetection, const int aModelInputWidth, const int aModelInputHeight);
    void removeLowConfidenceDetections(std::shared_ptr<DetectionOutput>& aDetections);
    void removeSimilarDetections(std::shared_ptr<DetectionOutput>& aDetections); 
   
};
#endif //ObjectDetectionHandler_H