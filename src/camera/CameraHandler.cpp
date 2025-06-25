
#include "CameraHandler.h"
#include "CameraFactory.h"
#include <opencv2/opencv.hpp>

#include "plog/Log.h"
#include <vector> 
#include "Utils.hpp"

CameraHandler::CameraHandler(const YAML::Node& aCameraConfig, 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, std::shared_ptr<NavDataHandler> aNavDataHandler) : 
    mFrameQueue(aFrameQueue), mNavDataHandler(aNavDataHandler), mRunning(true)
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
    LOGD << "Getting frames for camera with params: (fx fy cx cy near(m) far(m)): " 
         << aCameraCtx.mParams->mIntrinsics->focalX()       << ", "
         << aCameraCtx.mParams->mIntrinsics->focalY()       << ", "
         << aCameraCtx.mParams->mIntrinsics->centerX()      << ", " 
         << aCameraCtx.mParams->mIntrinsics->centerY()      << ", "
         << aCameraCtx.mParams->mIntrinsics->nearPlane_m    << ", "
         << aCameraCtx.mParams->mIntrinsics->farPlane_m; 

    // TODO: add something here to check if should stop or not
    while(isRunning())
    {
        aCameraCtx.mRate->start(); 

        CameraOutput frames = aCameraCtx.mCamera->getOutput();
        CameraFrame left = frames.left;  
        if (left.mFrame.empty()) 
        {
            LOGD << "Frame empty..."; 
            continue; 
        }

        // always resize the left image
        int width = aCameraCtx.mParams->mImgSize.first; 
        int height = aCameraCtx.mParams->mImgSize.second; 

        cv::resize(frames.left.mFrame,frames.left.mFrame, cv::Size(width, height)); 

        if(frames.isStereo)
        {
            // resize right frame
            cv::resize(frames.right.mFrame, frames.right.mFrame, cv::Size(width, height)); 
        }
        
        // get global pose of robot at timestamp of image 
        StampedCameraOutput output; 
        output.frames = frames; 
        cv::Matx44f T_G_V = mNavDataHandler->getClosestGlobalPose(frames.left.mTimestamp); 

        // compute pose of camera in global frame 
        output.T_G_C = T_G_V * aCameraCtx.mParams->mS2V; 
        output.mParams = aCameraCtx.mParams; 

        // push to frame queue 
        mFrameQueue->push(output);

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

    //parse s2v and pass to params 
    std::vector<float> xyz = aCameraConfig["xyz"].as<std::vector<float>>(); 
    std::vector<float> quat = aCameraConfig["quat"].as<std::vector<float>>(); 
    cv::Matx44f s2v = Utils::transformFromXYZQuat(xyz, quat); 

    std::vector<int> widthHeight = aCameraConfig["img_size"].as<std::vector<int>>();  
    std::pair<int, int> imgSize = {widthHeight[0], widthHeight[1]}; 
    
    auto params = std::make_shared<CameraParams>(intr, s2v, imgSize); 

    auto camCtx = CameraContext(rate, camera, params);

    mCameraContextMap.insert({id, camCtx}); 
}


