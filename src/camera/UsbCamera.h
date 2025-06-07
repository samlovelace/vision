#ifndef USBCAMERA_H
#define USBCAMERA_H
 
#include "ICamera.hpp" 

class UsbCamera : public ICamera
{ 
public:
    UsbCamera();
    ~UsbCamera();

    bool init() override; 
    CameraFrame getFrame() override; 

private:

    cv::VideoCapture mCapture; 
   
};
#endif //USBCAMERA_H