
#include "ObjectLocatorModule.h"
#include "ConfigManager.hpp"
#include "RosTopicManager.hpp"
#include "plog/Log.h"
#include "PointCloudViewer.h"
#include "Utils.hpp"
#include "vision_idl/msg/found_object_response.hpp"

ObjectLocatorModule::ObjectLocatorModule(const std::string& anObjectType) : 
    mVisualize(false), mSavePointCloud(true), mRunning(true)
{
    // Save object type to locate 
    mObjectToLocate = anObjectType; 

    auto config = ConfigManager::get().getFullConfig(); 
    mVisualize = config["visualize"].as<bool>(); 
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

    RosTopicManager::getInstance()->createPublisher<vision_idl::msg::FoundObjectResponse>("/vision/found_object_response"); 
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
    // setRunning(true); 
    // mCameraHandler->setRunning(true); 
    // mDetector->setRunning(true); 
    // mPoseEstimationHandler->setRunning(true); 

    mThreads.emplace_back(&CameraHandler::run, mCameraHandler.get());
    mThreads.emplace_back(&ObjectDetectionHandler::run, mDetector.get()); 
    mThreads.emplace_back(&PoseEstimationHandler::run, mPoseEstimationHandler.get());

    mThreads.emplace_back(&ObjectLocatorModule::run, this); 

    if(mVisualize)
    {
        runVisualizer(); 
    }
    
}

void ObjectLocatorModule::run()
{
    while(isRunning())
    {
        std::vector<DetectedObject> foundObjs = mObjectManager->getObjects(mObjectToLocate); 

        if(foundObjs.empty())
        {
            continue; 
        }

        // TODO: validate the found object data in some way 


        // Inform that object has been found 
        for(auto& obj : foundObjs)
        {
            // convert internal data type to idl data type
            //TODO: helper function to convert to idl format
            vision_idl::msg::FoundObjectResponse foundObj; 
            std_msgs::msg::String type; 
            type.set__data(obj.class_label); 
            foundObj.set__object_type(type); 

            geometry_msgs::msg::PointStamped objCentroid_G;
            objCentroid_G.header.frame_id = "world"; 
            
            geometry_msgs::msg::Point pt; 
            pt.set__x(obj.global_centroid.x); 
            pt.set__y(obj.global_centroid.y); 
            pt.set__x(obj.global_centroid.z); 
            objCentroid_G.set__point(pt); 

            foundObj.set__obj_centroid_g(objCentroid_G); 

            sensor_msgs::msg::PointCloud2 pc; 
            foundObj.set__obj_points_g(pc); 

            // publish response
            RosTopicManager::getInstance()->publishMessage<vision_idl::msg::FoundObjectResponse>("/vision/found_object_response", foundObj); 
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
}

void ObjectLocatorModule::runVisualizer()
{

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Object Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    int cloudNum = 0;

    while (isRunning())
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

        if (mCloudVisQueue)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            if (mCloudVisQueue->try_pop(cloud) && cloud && !cloud->empty())
            {
                if (mSavePointCloud)
                {
                    std::string cloudFile = "clouds/object_cloud_" + std::to_string(cloudNum++) + ".ply";
                    Utils::savePointCloudAsPLY<pcl::PointXYZ>(cloud, cloudFile);
                }

                // TODO: probably need to update this to be better at updating certain clouds 
                if (!viewer->updatePointCloud(cloud, "cloud")) 
                {
                    viewer->addPointCloud(cloud, "cloud");
                    viewer->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                }
                    
            }

            viewer->spinOnce(10);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (cv::waitKey(1) == 'q')
            break;
    }

    cv::destroyAllWindows(); 
}



void ObjectLocatorModule::stop()
{
    LOGI << "Object Locator Module commanded to STOP"; 

    setRunning(false); 
    mCameraHandler->setRunning(false);
    mFrameQueue->clear(); 
    mDetector->setRunning(false); 
    mPoseEstimationHandler->setRunning(false); 

    for(auto& t : mThreads)
    {
        if(t.joinable())
        {
            t.join(); 
        }
    }

    LOGD << "All processing threads in ObjectLocatorModule are stopped!"; 
}