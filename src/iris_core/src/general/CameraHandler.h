#ifndef CAMERAHANDLER_H
#define CAMERAHANDLER_H

#include <yaml-cpp/yaml.h>
#include <mutex>

#include "iris_common/ConcurrentQueue.hpp"
#include "CameraData.hpp"
#include "NavDataHandler.h"

#include "iris_msgs/msg/camera_frame.hpp"

class CameraHandler
{
public:
    CameraHandler(const YAML::Node& aCameraConfig,
                  std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue,
                  std::shared_ptr<NavDataHandler> aNavDataHandler);
    ~CameraHandler();

    // subscriptions are established in the constructor; run() only exists to
    // keep the thread-per-handler pattern the owning Locator module already uses
    void run();

    bool isRunning() {std::lock_guard<std::mutex> lock(mRunningMutex); return mRunning; }
    void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunningMutex); mRunning = aFlag; }

private:

    void onFrameReceived(const iris_msgs::msg::CameraFrame::SharedPtr aMsg);

    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue;
    std::shared_ptr<NavDataHandler> mNavDataHandler;

    bool mRunning;
    std::mutex mRunningMutex;

};
#endif //CAMERAHANDLER_H
