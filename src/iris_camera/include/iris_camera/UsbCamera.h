#ifndef USBCAMERA_H
#define USBCAMERA_H
 
#include "iris_camera/ICamera.hpp" 

class UsbCamera : public ICamera
{ 
public:
    UsbCamera();
    ~UsbCamera();

    bool init() override; 
    bool fini() override; 
    CameraOutput getOutput() override; 

private:

    cv::VideoCapture mCapture; 
   
};
#endif //USBCAMERA_H