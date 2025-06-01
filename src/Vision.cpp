
#include "Vision.h"
#include "ConfigManager.hpp"

#include "plog/Log.h"

Vision::Vision()
{
    //******** CAMERA SETUP ******************/
    YAML::Node cameraConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("camera", cameraConfig))
    {
        throw std::invalid_argument("Missing camera configuration");
    }

    mFrameQueue = std::make_shared<ConcurrentQueue<cv::Mat>>(); 
    mCameraHandler = std::make_shared<CameraHandler>(cameraConfig, mFrameQueue);

    /****** Neural Network Detection Setup ********/
    YAML::Node modelsConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("models", modelsConfig))
    {
        throw std::invalid_argument("Missing models configuration");
    }

    // TODO: pass Detection queue once implemented 
    mDetector = std::make_shared<ModelHandler>(modelsConfig, mFrameQueue); 

}

Vision::~Vision()
{
    for(auto& t : mThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }

}

void Vision::start()
{
    mThreads.emplace_back(&CameraHandler::run, mCameraHandler.get());
    mThreads.emplace_back(&ModelHandler::run, mDetector.get()); 
}

void Vision::stop()
{

}