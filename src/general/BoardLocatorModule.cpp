
#include "BoardLocatorModule.h"
#include "ConfigManager.hpp"
#include "RosTopicManager.hpp"

BoardLocatorModule::BoardLocatorModule(const std::string& aBoardToFind)
{
    auto config = ConfigManager::get().getFullConfig(); 
    mVisualize = config["visualize"].as<bool>(); 
    
    //******** CAMERA SETUP ******************/
    YAML::Node camerasConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("cameras", camerasConfig))
    {
        throw std::runtime_error("Missing camera configuration");
    }

    mNavDataHandler = std::make_shared<NavDataHandler>(config["nav_topic"].as<std::string>());

    mFrameQueue = std::make_shared<ConcurrentQueue<StampedCameraOutput>>(); 
    mCameraHandler = std::make_shared<CameraHandler>(camerasConfig, mFrameQueue, mNavDataHandler);

    m2DVisQueue = nullptr; 
    if(mVisualize)
    {
        // TODO: revert to visualize april tag detections once implemented
        m2DVisQueue = std::make_shared<ConcurrentQueue<StampedCameraOutput>>(); 
    }

    // Arcuo Tag detector configuration 
    auto tagDetectionConfig = config["tag_detection"]; 
    mDetector = std::make_shared<ArucoBoardDetector>(tagDetectionConfig, mFrameQueue, m2DVisQueue); 
}

BoardLocatorModule::~BoardLocatorModule()
{
    for(auto& t : mThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }
}

void BoardLocatorModule::start()
{   
    setRunning(true); 
    mThreads.emplace_back(&CameraHandler::run, mCameraHandler.get()); 
    mThreads.emplace_back(&ArucoBoardDetector::run, mDetector.get()); 

    if(mVisualize)
    {
        runVisualizer(); 
    }
}

void BoardLocatorModule::stop()
{
    LOGI << "Board Locator Module commanded to STOP"; 

    //setRunning(false); 
    mCameraHandler->setRunning(false);
    mFrameQueue->clear(); 
    
    for(auto& t : mThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }

    LOGD << "All processing threads in BoardLocatorModule are stopped!"; 
}

void BoardLocatorModule::runVisualizer()
{
    while(isRunning())
    {
        if(m2DVisQueue)
        {
            StampedCameraOutput output; 
            if(m2DVisQueue->try_pop(output))
            {
                cv::Mat frame = output.frames.left.mFrame; 
                if(!frame.empty())
                {
                    cv::imshow("Frame", frame); 
                }
            }
        }

        if (cv::waitKey(1) == 'q')
            break;
    }

    cv::destroyAllWindows(); 
}