#ifndef MODELHANDLER_H
#define MODELHANDLER_H
 
#include <yaml-cpp/yaml.h>
#include "ModelContext.hpp"
 
class ModelHandler 
{ 
public:
    ModelHandler();
    ~ModelHandler();

    bool setupModels(YAML::Node& aModelsConfig);
    void init(); 
    void handleFrame(const cv::Mat& aFrame); 

    std::vector<cv::Rect> getModelDetections(); 

private:

    std::vector<ModelContext> mModels; 
   
};
#endif //MODELHANDLER_H