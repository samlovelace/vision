
#include "CameraHandler.h"
#include "CameraFactory.h"
#include <opencv2/opencv.hpp>

#include "plog/Log.h"
#include <vector> 

CameraHandler::CameraHandler(const YAML::Node& aCameraConfig, std::shared_ptr<ConcurrentQueue<CameraFrame>> aFrameQueue) : 
    mFrameQueue(aFrameQueue)
{
    LOGD << YAML::Dump(aCameraConfig); 

    for(const auto& camera : aCameraConfig)
    {
        parseCameraConfig(camera); 
    }

}

CameraHandler::~CameraHandler()
{
    for(auto& t : mCameraThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }
}

void CameraHandler::run()
{
    for(const auto& [id, ctx] : mCameraContextMap)
    {
        mCameraThreads.push_back(std::thread(&CameraHandler::runCamera, this, ctx)); 
    }
}

void CameraHandler::runCamera(const CameraContext aCameraCtx)
{
    aCameraCtx.mCamera->init(); 

    while(true)
    {
        aCameraCtx.mRate->start(); 

        cv::Mat frame = aCameraCtx.mCamera->getFrame(); 
        if (frame.empty()) 
        {
            LOGD << "Frame empty..."; 
            continue; 
        }

        CameraFrame camFrame(frame); 
        
        // push to master frame queue 
        mFrameQueue->push(camFrame);

        aCameraCtx.mRate->block(); 
    }

}

void CameraHandler::parseCameraConfig(const YAML::Node& aCameraConfig)
{
    int id = aCameraConfig["id"].as<int>(); 
    auto rate = std::make_shared<RateController>(aCameraConfig["rate"].as<int>()); 
    auto camera = CameraFactory::create(aCameraConfig["type"].as<std::string>());

    if(nullptr == camera)
    {
        throw std::invalid_argument("Invalid camera configuration");  
    }
    
    std::vector<float> focal = aCameraConfig["focal"].as<std::vector<float>>(); 
    std::vector<float> center = aCameraConfig["center"].as<std::vector<float>>(); 
    float near = aCameraConfig["near"].as<float>(); 
    float far = aCameraConfig["far"].as<float>(); 

    auto intr = std::make_shared<CameraIntrinsics>(focal, center, near, far); 

    //TODO: parse s2v and pass to params 
    auto params = std::make_shared<CameraParams>(intr); 

    auto camCtx = CameraContext(rate, camera, params); 

    mCameraContextMap.insert({id, camCtx}); 
}


