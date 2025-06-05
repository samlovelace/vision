
#include "Vision.h"
#include "ConfigManager.hpp"

#include "plog/Log.h"

Vision::Vision() : mVisualize(true)
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

    mInferenceHandler = std::make_shared<InferenceHandler>(modelsConfig); 

    mDetectionQueue = std::make_shared<ConcurrentQueue<Detection>>();
    m2DVisQueue = std::make_shared<ConcurrentQueue<Detection>>(); 
    mDetector = std::make_shared<ObjectDetectionHandler>(modelsConfig, mFrameQueue, mDetectionQueue, m2DVisQueue, mInferenceHandler); 

    YAML::Node poseEstConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("pose_estimation", poseEstConfig))
    {
        throw std::invalid_argument("Missing or invalid pose_estimation config"); 
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

    // TODO: get this rate from somewhere, maybe match camera rate? 
    RateController rate(5); 

    while(true)
    {
        rate.start(); 

        if(mVisualize)
        {
            Detection detection; 
            if(m2DVisQueue->try_pop(detection))
            {
                if(detection.mFrame.empty())
                {
                    continue; 
                }

                cv::imshow("Detections", detection.mFrame); 
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
    
        rate.block();
    } 
}

void Vision::stop()
{

}