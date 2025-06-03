
#include "InferenceHandler.h"
#include "EngineFactory.h"
#include "ModelFactory.h"
#include "plog/Log.h"

InferenceHandler::InferenceHandler(const YAML::Node& aModelsConfig)
{
    for (const auto& modelConfig : aModelsConfig) {
        std::string modelType = modelConfig["name"].as<std::string>();
        std::string type = modelConfig["type"].as<std::string>();
        std::string path = modelConfig["path"].as<std::string>();
        std::string engineType = modelConfig["engine"].as<std::string>();

        // TODO: validate the configuration for model/engine pair 
        //if(!validateConfiguration())
        //{
        //  return false; 
        //}

        auto engine = EngineFactory::create(engineType); 
        auto model = ModelFactory::create(modelType); 
        
        // TODO: not sure this is the best place to put this? 
        engine->loadModel(path); 
        engine->init(); 
        model->init(); 

        ModelContext ctx(std::move(model), std::move(engine)); 
        mModels.insert({type, std::move(ctx)}); 
        LOGD << "Inserted Model: " << modelType << " Type: " << type; 
    }

}

InferenceHandler::~InferenceHandler()
{

}

std::shared_ptr<IModelOutput> InferenceHandler::runInference(const std::string& aType, const cv::Mat aFrame)
{
    std::shared_ptr<IModelOutput> modelOutput = nullptr; 

    if(mModels.find(aType) == mModels.end())
    {
        // model type not in map, cant do inference
        return modelOutput;  
    }

    cv::Mat preProcFrame; 
    const auto& ctx = mModels.at(aType);

    LOGD << "Running inference for " << aType; 

    ctx.mModel->preprocess(aFrame, preProcFrame); 
    auto output = ctx.mEngine->doInference(preProcFrame); 
    ctx.mModel->postprocess(output, modelOutput);
    
    LOGD << "Inference completed for " << aType; 

    return modelOutput; 
}
