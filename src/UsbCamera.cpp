
#include "UsbCamera.h"
#include "plog/Log.h"

UsbCamera::UsbCamera()
{

}

UsbCamera::~UsbCamera()
{

}

bool UsbCamera::init()
{

}

cv::Mat UsbCamera::getFrame()
{
    cv::Mat empty; 
    LOGD << "Getting frame"; 
    return empty; 
}