#ifndef MODELHANDLER_H
#define MODELHANDLER_H
 
#include <yaml-cpp/yaml.h>
#include "ModelContext.hpp"
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
 
class ModelHandler 
{ 
public:
    ModelHandler(const YAML::Node& aModelsConfig, 
                 std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                 std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue);

    ~ModelHandler();

    void run(); 

    void handleFrame(const cv::Mat& aFrame); 

    std::vector<cv::Rect> getModelDetections(); 

private:

    std::vector<ModelContext> mModels; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mFrameQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> mVisQueue; 

    void renderDetections(Detection& aDetection);
   
};
#endif //MODELHANDLER_H