
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

cv::Mat UsbCamera::getFrame()
{
    cv::Mat frame; 
    mCapture >> frame; 
    return frame; 
}