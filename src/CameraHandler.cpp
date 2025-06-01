
#include "CameraHandler.h"
#include "CameraFactory.h"
#include <opencv2/opencv.hpp>

#include "plog/Log.h"

CameraHandler::CameraHandler(const YAML::Node& aCameraConfig, std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue) : 
        mFrameQueue(aFrameQueue), mRate(nullptr)
{
    LOGD << YAML::Dump(aCameraConfig); 

    mRate = std::make_unique<RateController>(aCameraConfig["rate"].as<int>()); 
    mVisualize = aCameraConfig["visualize"].as<bool>(); 

    mCamera = CameraFactory::create(aCameraConfig["type"].as<std::string>()); 
    if(nullptr == mCamera)
    {
        throw std::invalid_argument("Invalid camera configuration");  
    }

}

CameraHandler::~CameraHandler()
{
}

void CameraHandler::run()
{
    mCamera->init(); 

    while(true)
    {
        mRate->start(); 

        cv::Mat frame = mCamera->getFrame(); 
        if (frame.empty()) 
        {
            LOGD << "Frame empty..."; 
            continue; 
        }

        mFrameQueue->push(frame);

        mRate->block(); 
    }

}


