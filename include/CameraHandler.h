#ifndef CAMERAHANDLER_H
#define CAMERAHANDLER_H

#include <yaml-cpp/yaml.h>

#include "ICamera.hpp"
#include "RateController.h"
#include "ConcurrentQueue.hpp"
 
class CameraHandler 
{ 
public:
    CameraHandler(const YAML::Node& aCameraConfig, std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue);
    ~CameraHandler();

    void run(); 

private:
    std::shared_ptr<RateController> mRate; 
    std::shared_ptr<ICamera> mCamera; 

    std::shared_ptr<ConcurrentQueue<cv::Mat>> mFrameQueue; 

    bool mVisualize; 
   
};
#endif //CAMERAHANDLER_H