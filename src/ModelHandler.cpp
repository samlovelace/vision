
#include "ModelHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

#include <opencv2/opencv.hpp>

ModelHandler::ModelHandler(const YAML::Node& aModelsConfig, 
                           std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue, 
                           std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                           std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue) : 
    mFrameQueue(aFrameQueue), mDetectionQueue(aDetectionQueue), mVisQueue(aVisQueue)
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
            cv::Mat preProcFrame; 
            Detection detection;

            // TODO: pushing to the queue wont behavior properly when there are multiple models 
            for(auto& ctx : mModels)
            {
                cv::Mat preProcFrame; 

                ctx.mModel->preprocess(frame, preProcFrame); 
                cv::Mat output = ctx.mEngine->doInference(preProcFrame); 
                ctx.mModel->postprocess(output, detection.mDetections); 
            }

            // set the frame that the model(s) ran inference on
            detection.mFrame = frame; 

            // push to queues for 3d estimator
            mDetectionQueue->push(detection); 

            renderDetections(detection); 
            mVisQueue->push(detection); 
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

void ModelHandler::renderDetections(Detection& aDetection)
{
    for(const auto& bbox : aDetection.mDetections)
    {
        cv::rectangle(aDetection.mFrame, bbox, cv::Scalar(0, 255, 0, 0), 1, 8); 
    }
}

std::vector<cv::Rect> ModelHandler::getModelDetections()
{
    // TODO: make this more general purpose
    //       Need some way of keying the model contexts
    return mModels[0].mDetections; 
}



