
#include "CameraHandler.h"
#include "CameraFactory.h"
#include <opencv2/opencv.hpp>

#include "plog/Log.h"

CameraHandler::CameraHandler(std::shared_ptr<ModelHandler> mh) : mModelHandler(mh), mRate(nullptr), mVisualize(false)
{

}

CameraHandler::~CameraHandler()
{
}

bool CameraHandler::init(const YAML::Node& aCameraConfig)
{
    LOGD << YAML::Dump(aCameraConfig); 

    mRate = std::make_unique<RateController>(aCameraConfig["rate"].as<int>()); 
    mVisualize = aCameraConfig["visualize"].as<bool>(); 

    mCamera = CameraFactory::create(aCameraConfig["type"].as<std::string>()); 
    if(nullptr == mCamera)
    {
        return false; 
    }

    return true; 
}

void CameraHandler::run()
{
    mCamera->init(); 

    while(true)
    {
        mRate->start(); 

        cv::Mat frame = mCamera->getFrame(); 
        
        if(mVisualize)
        {
            if (frame.empty()) 
            {
                continue; 
            }
            
            cv::imshow("Live Webcam Feed", frame);  // Show the frame
        }

        mModelHandler->handleFrame(frame); 
        
        // Exit on 'q' key press
        if (cv::waitKey(1) == 'q') {
            std::cout << "✅ Quitting...\n";
            break;
        }

        mRate->block(); 
    }

}
