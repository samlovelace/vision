
#include "CameraHandler.h"
#include "iris_common/RosTopicManager.hpp"
#include "Utils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "plog/Log.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{
    std::chrono::system_clock::time_point toChrono(const builtin_interfaces::msg::Time& aStamp)
    {
        auto duration = std::chrono::seconds(aStamp.sec) + std::chrono::nanoseconds(aStamp.nanosec);
        return std::chrono::system_clock::time_point(duration);
    }

    CameraFrame toCameraFrame(const sensor_msgs::msg::Image& anImage, const std::string& anEncoding)
    {
        return CameraFrame(cv_bridge::toCvCopy(anImage, anEncoding)->image, toChrono(anImage.header.stamp));
    }
}

CameraHandler::CameraHandler(const YAML::Node& aCameraConfig,
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, std::shared_ptr<NavDataHandler> aNavDataHandler) :
    mFrameQueue(aFrameQueue), mNavDataHandler(aNavDataHandler), mRunning(true)
{
    LOGD << YAML::Dump(aCameraConfig);
    std::string packagePath = ament_index_cpp::get_package_share_directory("iris_bringup");
    std::string configPath = packagePath + "/configuration/camera/";

    for(const auto& cameraFile : aCameraConfig)
    {
        if(cameraFile["file"])
        {
            YAML::Node camera = YAML::LoadFile(configPath + cameraFile["file"].as<std::string>());
            int id = camera["id"].as<int>();
            std::string topicName = "camera/cam" + std::to_string(id) + "/frame";

            RosTopicManager::getInstance()->createSubscriber<iris_msgs::msg::CameraFrame>(topicName,
                std::bind(&CameraHandler::onFrameReceived, this, std::placeholders::_1));
        }
    }
}

CameraHandler::~CameraHandler()
{
}

void CameraHandler::run()
{
    // subscriptions were already established in the constructor - nothing to do here
}

void CameraHandler::onFrameReceived(const iris_msgs::msg::CameraFrame::SharedPtr aMsg)
{
    if(!isRunning())
    {
        return;
    }

    CameraOutput frames;
    frames.left = toCameraFrame(aMsg->left, "rgb8");
    frames.isStereo = aMsg->is_stereo;
    frames.isDepth = aMsg->is_depth;

    if(frames.isStereo)
    {
        frames.right = toCameraFrame(aMsg->right, "rgb8");
    }

    if(frames.isDepth)
    {
        frames.depth = toCameraFrame(aMsg->depth, "32FC1");
    }

    auto intr = std::make_shared<CameraIntrinsics>(aMsg->k[0], aMsg->k[4], aMsg->k[2], aMsg->k[5],
                                                    aMsg->near_m, aMsg->far_m, aMsg->baseline_m);

    std::vector<float> xyz = {static_cast<float>(aMsg->s2v.translation.x),
                               static_cast<float>(aMsg->s2v.translation.y),
                               static_cast<float>(aMsg->s2v.translation.z)};
    std::vector<float> quat = {static_cast<float>(aMsg->s2v.rotation.x),
                                static_cast<float>(aMsg->s2v.rotation.y),
                                static_cast<float>(aMsg->s2v.rotation.z),
                                static_cast<float>(aMsg->s2v.rotation.w)};
    cv::Matx44f s2v = Utils::transformFromXYZQuat(xyz, quat);

    std::pair<int, int> imgSize = {static_cast<int>(aMsg->img_width), static_cast<int>(aMsg->img_height)};
    auto params = std::make_shared<CameraParams>(intr, s2v, imgSize);

    // get global pose of robot at timestamp of image
    StampedCameraOutput output;
    output.frames = frames;
    cv::Matx44f T_G_V = mNavDataHandler->getClosestGlobalPose(frames.left.mTimestamp);

    // compute pose of camera in global frame
    output.T_G_C = T_G_V * params->mS2V;
    output.mParams = params;

    mFrameQueue->push(output);
}
