
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
    mCapture.open(0); 

    if(!mCapture.isOpened())
    {
        return false; 
    }

    mCapture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
}

CameraOutput UsbCamera::getOutput()
{
    cv::Mat img; 
    mCapture >> img; 
    auto now = std::chrono::system_clock::now(); 
    CameraFrame frame(img, now); 

    CameraOutput out; 
    out.isStereo = false; 
    out.left = frame; 

    return out; 
}