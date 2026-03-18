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
    bool hasModelType(const std::string& aType); 
    std::pair<int, int> getModelInputSize(const std::string& aType);

private:

    std::map<std::string, ModelContext> mModels; 
   
};
#endif //INFERENCEHANDLER_H