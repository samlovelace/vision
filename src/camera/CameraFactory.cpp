
#include "CameraFactory.h"
#include "UsbCamera.h"
#include "RosCamera.h"
#include "RealSenseCamera_d435i.h"

#include "plog/Log.h"

std::shared_ptr<ICamera> CameraFactory::create(const std::string& aCamType)
{
    if("webcam" == aCamType || "usb" == aCamType)
    {
        return std::make_shared<UsbCamera>(); 
    }
    else if ("ros" == aCamType)
    {
        return std::make_shared<RosCamera>(); 
    }
    else if ("rs-d435i" == aCamType)
    {
        return std::make_shared<RealSenseCamera_d435i>(); 
    }
    else{
        LOGE << "Unsupported camera type: " << aCamType; 
        return nullptr; 
    }
}