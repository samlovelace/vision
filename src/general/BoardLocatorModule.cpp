
#include "BoardLocatorModule.h"
#include "ConfigManager.hpp"
#include "RosTopicManager.hpp"
#include "robot_idl/msg/found_object_response.hpp"

BoardLocatorModule::BoardLocatorModule(const KnownObjectConfig& anObjectToFind) 
    : mObjectToFind(anObjectToFind)
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

    mObjectManager = std::make_shared<DetectedObjectManager>(); 
    mDetector = std::make_shared<ArucoBoardDetector>(anObjectToFind, mFrameQueue, m2DVisQueue, mObjectManager); 

    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::FoundObjectResponse>("vision/response"); 
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
    mThreads.emplace_back(&BoardLocatorModule::run, this); 

    if(mVisualize)
    {
        runVisualizer(); 
    }
}

void BoardLocatorModule::run()
{
    while(isRunning())
    {
        std::vector<DetectedObject> foundObjs = mObjectManager->getObjects(mObjectToFind.mType); 

        if(foundObjs.empty())
        {
            continue; 
        }

        // TODO: validate the found object data in some way 
        // TODO: dont keep sending the same object if pose doesnt change significantly 

        // Inform that object has been found 
        for(auto& obj : foundObjs)
        {
            // convert internal data type to idl data type
            //TODO: helper function to convert to idl format
            robot_idl::msg::FoundObjectResponse foundObj; 
            std_msgs::msg::String type; 
            type.set__data(obj.class_label); 
            foundObj.set__object_type(type); 

            geometry_msgs::msg::PoseStamped objPose_G;
            objPose_G.header.frame_id = "world"; 
            
            geometry_msgs::msg::Point pt; 
            pt.set__x(obj.global_centroid.x); 
            pt.set__y(obj.global_centroid.y); 
            pt.set__z(obj.global_centroid.z); 
            objPose_G.pose.set__position(pt); 

            geometry_msgs::msg::Quaternion quat; 
            quat.set__w(obj.global_orientation[0]);
            quat.set__x(obj.global_orientation[1]);
            quat.set__y(obj.global_orientation[2]);
            quat.set__z(obj.global_orientation[3]); 
            objPose_G.pose.set__orientation(quat); 

            foundObj.set__obj_pose_g(objPose_G); 

            sensor_msgs::msg::PointCloud2 pc; 
            foundObj.set__obj_points_g(pc); 

            // publish response
            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::FoundObjectResponse>("vision/response", foundObj); 
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
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