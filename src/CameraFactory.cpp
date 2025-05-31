
#include "CameraFactory.h"
#include "UsbCamera.h"

#include "plog/Log.h"

std::shared_ptr<ICamera> CameraFactory::create(const std::string& aCamType)
{
    if("webcam" == aCamType || "usb" == aCamType)
    {
        return std::make_shared<UsbCamera>(); 
    }
    else{
        LOGE << "Unsupported camera type: " << aCamType; 
        return nullptr; 
    }

}