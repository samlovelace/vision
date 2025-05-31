
#include "ModelHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

ModelHandler::ModelHandler()
{

}

ModelHandler::~ModelHandler()
{

}

bool ModelHandler::setupModels(YAML::Node& aModelsConfig)
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

        ModelContext ctx(std::move(model), std::move(engine)); 

        mModels.push_back(std::move(ctx)); 
    }

    return true; 
}

void ModelHandler::test()
{
    for(const auto& ctx : mModels)
    {
        ctx.mModel->init(); 
        ctx.mEngine->init(); 
    }

}

void ModelHandler::handleFrame(const cv::Mat& aFrame)
{
    LOGD << "passing frame to models"; 
}

