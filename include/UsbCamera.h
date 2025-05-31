#ifndef USBCAMERA_H
#define USBCAMERA_H
 
#include "ICamera.hpp" 
#include <opencv2/opencv.hpp>

class UsbCamera : public ICamera
{ 
public:
    UsbCamera();
    ~UsbCamera();

    bool init() override; 
    cv::Mat getFrame() override; 

private:

    cv::VideoCapture mCapture; 
   
};
#endif //USBCAMERA_H