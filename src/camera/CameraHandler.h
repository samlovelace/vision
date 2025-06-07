#ifndef CAMERAHANDLER_H
#define CAMERAHANDLER_H

#include <yaml-cpp/yaml.h>
#include <thread> 

#include "ICamera.hpp"
#include "RateController.h"
#include "ConcurrentQueue.hpp"

#include "CameraContext.hpp"
#include "CameraFrame.hpp"
 
class CameraHandler 
{ 
public:
    CameraHandler(const YAML::Node& aCameraConfig, std::shared_ptr<ConcurrentQueue<CameraFrame>> aFrameQueue);
    ~CameraHandler();

    void run();

private:

    std::shared_ptr<ConcurrentQueue<CameraFrame>> mFrameQueue;
    std::map<int, CameraContext> mCameraContextMap;
    
    void runCamera(const CameraContext aCamCtx); 
    void parseCameraConfig(const YAML::Node& aCameraConfig); 

    std::vector<std::thread> mCameraThreads; 
   
};
#endif //CAMERAHANDLER_H