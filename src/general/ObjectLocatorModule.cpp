
#include "ObjectLocatorModule.h"
#include "ConfigManager.hpp"
#include "RosTopicManager.hpp"
#include "plog/Log.h"
#include "PointCloudViewer.h"
#include "Utils.hpp"

ObjectLocatorModule::ObjectLocatorModule(const std::string& anObjectType) : 
    mVisualize(false), mVisualizePointCloud(true), mSavePointCloud(true)
{
    auto config = ConfigManager::get().getFullConfig(); 
    mVisualize = config["visualize"].as<bool>(); 
    mVisualizePointCloud = config["visualize_cloud"].as<bool>(); 
    mSavePointCloud = config["save_clouds"].as<bool>(); 

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

    mCloudVisQueue = std::make_shared<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>>(); 

    mDepthFrameVisQueue = std::make_shared<ConcurrentQueue<cv::Mat>>(); 
    mObjectManager = std::make_shared<DetectedObjectManager>(); 
    mPoseEstimationHandler = std::make_shared<PoseEstimationHandler>(poseEstConfig, 
                                                                     mDetectionQueue, 
                                                                     mInferenceHandler, 
                                                                     mDepthFrameVisQueue, 
                                                                     mCloudVisQueue, 
                                                                     mObjectManager); 
}

ObjectLocatorModule::~ObjectLocatorModule()
{
    for(auto& t : mThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }

}

void ObjectLocatorModule::start()
{
    mThreads.emplace_back(&CameraHandler::run, mCameraHandler.get());
    mThreads.emplace_back(&ObjectDetectionHandler::run, mDetector.get()); 
    mThreads.emplace_back(&PoseEstimationHandler::run, mPoseEstimationHandler.get());

    if(mVisualize)
        mThreads.emplace_back(&ObjectLocatorModule::runVisualizer, this); 
    
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }
}

void ObjectLocatorModule::runVisualizer()
{
    PointCloudViewer pcViewer;
    if (mVisualizePointCloud) 
        pcViewer.start(); 

    int cloudNum = 0;

    while (true)
    {
        if (m2DVisQueue)
        {
            Detection detection;
            if (m2DVisQueue->try_pop(detection))
            {
                cv::Mat leftFrame = detection.mCameraOutput.frames.left.mFrame;
                if (!leftFrame.empty())
                    cv::imshow("Detections", leftFrame);
            }
        }

        if (mDepthFrameVisQueue)
        {
            cv::Mat depthFrame;
            if (mDepthFrameVisQueue->try_pop(depthFrame))
            {
                if (!depthFrame.empty())
                    cv::imshow("DepthMap", depthFrame);
            }
        }

        if (mCloudVisQueue && (mVisualizePointCloud || mSavePointCloud))
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            if (mCloudVisQueue->try_pop(cloud) && cloud && !cloud->empty())
            {
                if (mSavePointCloud)
                {
                    std::string cloudFile = "clouds/object_cloud_" + std::to_string(cloudNum++) + ".ply";
                    Utils::savePointCloudAsPLY<pcl::PointXYZ>(cloud, cloudFile);
                }

                if (mVisualizePointCloud)
                    pcViewer.updateCloud(cloud);
            }
        }

        if (cv::waitKey(1) == 'q')
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



void ObjectLocatorModule::stop()
{

}