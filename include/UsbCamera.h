#ifndef USBCAMERA_H
#define USBCAMERA_H
 
#include "ICamera.hpp" 

class UsbCamera : public ICamera
{ 
public:
    UsbCamera();
    ~UsbCamera();

    bool init() override; 
    cv::Mat getFrame() override; 

private:
   
};
#endif //USBCAMERA_H