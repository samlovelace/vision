
#include "Vision.h"
#include "ConfigManager.hpp"
#include "RosTopicManager.hpp"
#include "plog/Log.h"

Vision::Vision() : mVisualize(false)
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

    /****** Neural Network Detection Setup ********/
    YAML::Node modelsConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("models", modelsConfig))
    {
        throw std::runtime_error("Missing models configuration");
    }

    mInferenceHandler = std::make_shared<InferenceHandler>(modelsConfig); 
    mDetectionQueue = std::make_shared<ConcurrentQueue<Detection>>();
    
    m2DVisQueue = nullptr; 
    if(mVisualize)
    {
        m2DVisQueue = std::make_shared<ConcurrentQueue<Detection>>(); 
    }

    YAML::Node detectionConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("object_detection", detectionConfig))
    {
        throw std::runtime_error("invalid object_detection config"); 
    }
    
    mDetector = std::make_shared<ObjectDetectionHandler>(detectionConfig, mFrameQueue, mDetectionQueue, m2DVisQueue, mInferenceHandler); 

    YAML::Node poseEstConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("pose_estimation", poseEstConfig))
    {
        throw std::runtime_error("Missing or invalid pose_estimation config"); 
    }

    mDepthFrameVisQueue = std::make_shared<ConcurrentQueue<cv::Mat>>(); 
    mPoseEstimationHandler = std::make_shared<PoseEstimationHandler>(poseEstConfig, mDetectionQueue, mInferenceHandler, mDepthFrameVisQueue); 
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
    mThreads.emplace_back(&ObjectDetectionHandler::run, mDetector.get()); 
    mThreads.emplace_back(&PoseEstimationHandler::run, mPoseEstimationHandler.get());

    while(true)
    {
        if(mVisualize)
        {
            Detection detection; 
            if(m2DVisQueue->try_pop(detection))
            {
                cv::Mat leftFrame = detection.mCameraOutput.frames.left.mFrame; 
                if(leftFrame.empty())
                {
                    continue; 
                }

                cv::imshow("Detections", leftFrame); 
            }

            cv::Mat depthFrame; 
            if(mDepthFrameVisQueue->try_pop(depthFrame))
            {
                if(depthFrame.empty())
                {
                    continue; 
                }

                cv::imshow("DepthMap", depthFrame); 
            }
        }

        if(cv::waitKey(1) == 'q')
        {
            break; 
        }
    } 
}

void Vision::stop()
{

}