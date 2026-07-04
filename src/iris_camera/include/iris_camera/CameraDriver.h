#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

#include <yaml-cpp/yaml.h>
#include <thread>
#include <map>
#include <mutex>

#include "iris_camera/ICamera.hpp"
#include "iris_camera/RateController.h"
#include "iris_camera/CameraContext.hpp"
#include "iris_camera/CameraData.hpp"

class CameraDriver
{
public:
    CameraDriver(const YAML::Node& aCameraConfig);
    ~CameraDriver();

    void run();

    bool isRunning() {std::lock_guard<std::mutex> lock(mRunningMutex); return mRunning; }
    void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunningMutex); mRunning = aFlag; }

private:

    void runCamera(const CameraContext aCameraCtx, int aCameraId, const std::string& aTopicName);
    void parseCameraConfig(const YAML::Node& aCameraConfig);

    std::map<int, CameraContext> mCameraContextMap;
    std::map<int, std::string> mCameraTopicMap;

    std::vector<std::thread> mCameraThreads;

    bool mRunning;
    std::mutex mRunningMutex;

};
#endif //CAMERADRIVER_H
