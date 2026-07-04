#include "iris_camera/CameraDriver.h"
#include "iris_camera/CameraFactory.h"
#include "iris_camera/CameraParams.hpp"
#include "iris_common/RosTopicManager.hpp"

#include "iris_msgs/msg/camera_frame.hpp"
#include <cv_bridge/cv_bridge.h>

#include "plog/Log.h"
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{
    // local copy of Utils::transformFromXYZQuat (avoids pulling in iris_core's
    // PCL-dependent Utils.hpp just for this one function)
    cv::Matx44f transformFromXYZQuat(const std::vector<float>& xyz, const std::vector<float>& quat)
    {
        float x = xyz[0];
        float y = xyz[1];
        float z = xyz[2];

        float qx = quat[0];
        float qy = quat[1];
        float qz = quat[2];
        float qw = quat[3];

        float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

        float xx = qx * qx;
        float yy = qy * qy;
        float zz = qz * qz;
        float xy = qx * qy;
        float xz = qx * qz;
        float yz = qy * qz;
        float wx = qw * qx;
        float wy = qw * qy;
        float wz = qw * qz;

        cv::Matx44f mat = cv::Matx44f::eye();

        mat(0,0) = 1.0f - 2.0f * (yy + zz);
        mat(0,1) = 2.0f * (xy - wz);
        mat(0,2) = 2.0f * (xz + wy);
        mat(0,3) = x;

        mat(1,0) = 2.0f * (xy + wz);
        mat(1,1) = 1.0f - 2.0f * (xx + zz);
        mat(1,2) = 2.0f * (yz - wx);
        mat(1,3) = y;

        mat(2,0) = 2.0f * (xz - wy);
        mat(2,1) = 2.0f * (yz + wx);
        mat(2,2) = 1.0f - 2.0f * (xx + yy);
        mat(2,3) = z;

        return mat;
    }

    // matrix -> geometry_msgs/Transform (rotation via standard trace method)
    geometry_msgs::msg::Transform toTransformMsg(const cv::Matx44f& T)
    {
        float r00 = T(0,0), r01 = T(0,1), r02 = T(0,2);
        float r10 = T(1,0), r11 = T(1,1), r12 = T(1,2);
        float r20 = T(2,0), r21 = T(2,1), r22 = T(2,2);

        float trace = r00 + r11 + r22;
        float qw, qx, qy, qz;

        if (trace > 0.0f) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            qw = 0.25f / s;
            qx = (r21 - r12) * s;
            qy = (r02 - r20) * s;
            qz = (r10 - r01) * s;
        } else if (r00 > r11 && r00 > r22) {
            float s = 2.0f * std::sqrt(1.0f + r00 - r11 - r22);
            qw = (r21 - r12) / s;
            qx = 0.25f * s;
            qy = (r01 + r10) / s;
            qz = (r02 + r20) / s;
        } else if (r11 > r22) {
            float s = 2.0f * std::sqrt(1.0f + r11 - r00 - r22);
            qw = (r02 - r20) / s;
            qx = (r01 + r10) / s;
            qy = 0.25f * s;
            qz = (r12 + r21) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + r22 - r00 - r11);
            qw = (r10 - r01) / s;
            qx = (r02 + r20) / s;
            qy = (r12 + r21) / s;
            qz = 0.25f * s;
        }

        geometry_msgs::msg::Transform t;
        t.translation.x = T(0,3);
        t.translation.y = T(1,3);
        t.translation.z = T(2,3);
        t.rotation.x = qx;
        t.rotation.y = qy;
        t.rotation.z = qz;
        t.rotation.w = qw;
        return t;
    }

    builtin_interfaces::msg::Time toRosTime(const std::chrono::system_clock::time_point& aTime)
    {
        auto duration = aTime.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec);

        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(sec.count());
        stamp.nanosec = static_cast<uint32_t>(nsec.count());
        return stamp;
    }

    iris_msgs::msg::CameraFrame buildCameraFrameMsg(const CameraOutput& aFrames, const CameraParams& aParams, int aCameraId)
    {
        iris_msgs::msg::CameraFrame msg;

        msg.header.stamp = toRosTime(aFrames.left.mTimestamp);
        msg.header.frame_id = std::to_string(aCameraId);

        msg.left = *cv_bridge::CvImage(msg.header, "rgb8", aFrames.left.mFrame).toImageMsg();

        msg.is_stereo = aFrames.isStereo;
        if (aFrames.isStereo)
        {
            msg.right = *cv_bridge::CvImage(msg.header, "rgb8", aFrames.right.mFrame).toImageMsg();
        }

        msg.is_depth = aFrames.isDepth;
        if (aFrames.isDepth)
        {
            msg.depth = *cv_bridge::CvImage(msg.header, "32FC1", aFrames.depth.mFrame).toImageMsg();
        }

        const CameraIntrinsics& intr = *aParams.mIntrinsics;
        msg.k = {intr.focalX(), 0.0f, intr.centerX(),
                 0.0f, intr.focalY(), intr.centerY(),
                 0.0f, 0.0f, 1.0f};
        msg.near_m = intr.nearPlane_m;
        msg.far_m = intr.farPlane_m;
        msg.baseline_m = intr.baseline_m;
        msg.img_width = static_cast<uint32_t>(aParams.mImgSize.first);
        msg.img_height = static_cast<uint32_t>(aParams.mImgSize.second);

        msg.s2v = toTransformMsg(aParams.mS2V);

        return msg;
    }
}

CameraDriver::CameraDriver(const YAML::Node& aCameraConfig) : mRunning(true)
{
    std::string packagePath = ament_index_cpp::get_package_share_directory("iris_bringup");
    std::string configPath = packagePath + "/configuration/camera/";

    for(const auto& cameraFile : aCameraConfig)
    {
        if(cameraFile["file"])
        {
            YAML::Node camera = YAML::LoadFile(configPath + cameraFile["file"].as<std::string>());
            parseCameraConfig(camera);
        }
    }
}

CameraDriver::~CameraDriver()
{
    for(auto& t : mCameraThreads)
    {
        if(t.joinable())
        {
            t.join();
        }
    }
}

void CameraDriver::run()
{
    for(const auto& [id, ctx] : mCameraContextMap)
    {
        mCameraThreads.push_back(std::thread(&CameraDriver::runCamera, this, ctx, id, mCameraTopicMap.at(id)));
    }
}

void CameraDriver::runCamera(const CameraContext aCameraCtx, int aCameraId, const std::string& aTopicName)
{
    aCameraCtx.mCamera->init();

    LOGD << "Getting frames for camera with params: (fx fy cx cy near(m) far(m)): "
         << aCameraCtx.mParams->mIntrinsics->focalX()       << ", "
         << aCameraCtx.mParams->mIntrinsics->focalY()       << ", "
         << aCameraCtx.mParams->mIntrinsics->centerX()      << ", "
         << aCameraCtx.mParams->mIntrinsics->centerY()      << ", "
         << aCameraCtx.mParams->mIntrinsics->nearPlane_m    << ", "
         << aCameraCtx.mParams->mIntrinsics->farPlane_m;

    while(isRunning())
    {
        aCameraCtx.mRate->start();

        CameraOutput frames = aCameraCtx.mCamera->getOutput();
        if (frames.left.mFrame.empty())
        {
            LOGD << "Frame empty...";
            aCameraCtx.mRate->block();
            continue;
        }

        if(frames.isStereo)
        {
            int width = aCameraCtx.mParams->mImgSize.first;
            int height = aCameraCtx.mParams->mImgSize.second;
            cv::resize(frames.right.mFrame, frames.right.mFrame, cv::Size(width, height));
        }

        iris_msgs::msg::CameraFrame msg = buildCameraFrameMsg(frames, *aCameraCtx.mParams, aCameraId);
        RosTopicManager::getInstance()->publishMessage<iris_msgs::msg::CameraFrame>(aTopicName, msg);

        aCameraCtx.mRate->block();
    }

    aCameraCtx.mCamera->fini();
    LOGD << "CameraDriver processing loop exited";
}

void CameraDriver::parseCameraConfig(const YAML::Node& aCameraConfig)
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
    float baseline = aCameraConfig["baseline"].as<float>();

    auto intr = std::make_shared<CameraIntrinsics>(focal, center, near, far, baseline);

    std::vector<float> xyz = aCameraConfig["xyz"].as<std::vector<float>>();
    std::vector<float> quat = aCameraConfig["quat"].as<std::vector<float>>();
    cv::Matx44f s2v = transformFromXYZQuat(xyz, quat);

    std::vector<int> widthHeight = aCameraConfig["img_size"].as<std::vector<int>>();
    std::pair<int, int> imgSize = {widthHeight[0], widthHeight[1]};

    auto params = std::make_shared<CameraParams>(intr, s2v, imgSize);
    auto camCtx = CameraContext(rate, camera, params);

    mCameraContextMap.insert({id, camCtx});

    std::string topicName = "camera/cam" + std::to_string(id) + "/frame";
    mCameraTopicMap.insert({id, topicName});
    RosTopicManager::getInstance()->createPublisher<iris_msgs::msg::CameraFrame>(topicName);
}
