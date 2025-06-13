#ifndef CAMERAHANDLER_H
#define CAMERAHANDLER_H

#include <yaml-cpp/yaml.h>
#include <thread> 

#include "ICamera.hpp"
#include "RateController.h"
#include "ConcurrentQueue.hpp"

#include "CameraContext.hpp"
#include "CameraData.hpp"
#include "NavDataHandler.h"
 
class CameraHandler 
{ 
public:
    CameraHandler(const YAML::Node& aCameraConfig, 
                  std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, 
                  std::shared_ptr<NavDataHandler> aNavDataHandler);
    ~CameraHandler();

    void run();

private:

    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue;
    std::map<int, CameraContext> mCameraContextMap;
    
    void runCamera(const CameraContext aCamCtx); 
    void parseCameraConfig(const YAML::Node& aCameraConfig); 

    std::vector<std::thread> mCameraThreads; 

    std::shared_ptr<NavDataHandler> mNavDataHandler; 
   
};
#endif //CAMERAHANDLER_H