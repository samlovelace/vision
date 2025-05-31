#ifndef CAMERAHANDLER_H
#define CAMERAHANDLER_H

#include "ICamera.hpp"
#include "RateController.h"

#include "ModelHandler.h"
 
class CameraHandler 
{ 
public:
    CameraHandler(std::shared_ptr<ModelHandler> mh);
    ~CameraHandler();

    bool init(const YAML::Node& aCameraConfig); 
    void run(); 


private:
    std::shared_ptr<RateController> mRate; 
    std::shared_ptr<ICamera> mCamera; 
    std::shared_ptr<ModelHandler> mModelHandler; 
   
};
#endif //CAMERAHANDLER_H