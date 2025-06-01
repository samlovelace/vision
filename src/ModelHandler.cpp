
#include "ModelHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

ModelHandler::ModelHandler(const YAML::Node& aModelsConfig, std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue) : 
    mFrameQueue(aFrameQueue)
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
        mModels.push_back(std::move(ctx)); 
    }

}

ModelHandler::~ModelHandler()
{

}

void ModelHandler::run()
{
    while(true)
    {
        cv::Mat frame; 
        if(mFrameQueue->pop(frame))
        {
            handleFrame(frame); 

            //TODO: push to Detection queue once implemented
        }
    }
}

void ModelHandler::handleFrame(const cv::Mat& aFrame)
{
    for(auto& ctx : mModels)
    {
        cv::Mat preProcFrame; 

        ctx.mModel->preprocess(aFrame, preProcFrame); 
        cv::Mat output = ctx.mEngine->doInference(preProcFrame); 
        ctx.mModel->postprocess(output, ctx.mDetections); 
    }
}

std::vector<cv::Rect> ModelHandler::getModelDetections()
{
    // TODO: make this more general purpose
    //       Need some way of keying the model contexts
    return mModels[0].mDetections; 
}



