
#include "CameraHandler.h"
#include "CameraFactory.h"

#include "plog/Log.h"

CameraHandler::CameraHandler(std::shared_ptr<ModelHandler> mh) : mModelHandler(mh)
{

}

CameraHandler::~CameraHandler()
{
}

bool CameraHandler::init(const YAML::Node& aCameraConfig)
{
    LOGD << YAML::Dump(aCameraConfig); 

    mRate = std::make_unique<RateController>(aCameraConfig["rate"].as<int>()); 

    mCamera = CameraFactory::create(aCameraConfig["type"].as<std::string>()); 
    if(nullptr == mCamera)
    {
        return false; 
    }

    return true; 
}

void CameraHandler::run()
{
    while(true)
    {
        mRate->start(); 

        cv::Mat frame = mCamera->getFrame(); 
        mModelHandler->handleFrame(frame); 
        
        mRate->block(); 

    }

}