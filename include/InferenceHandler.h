#ifndef INFERENCEHANDLER_H
#define INFERENCEHANDLER_H

#include <yaml-cpp/yaml.h>
#include <map> 
#include <ModelContext.hpp>
 
class InferenceHandler 
{ 
public:
    InferenceHandler(const YAML::Node& aModelsConfig);
    ~InferenceHandler();

    std::shared_ptr<IModelOutput> runInference(const std::string& aType, const cv::Mat aFrame);


private:

    std::map<std::string, ModelContext> mModels; 
   
};
#endif //INFERENCEHANDLER_H