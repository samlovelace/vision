#ifndef ICAMERA_HPP
#define ICAMERA_HPP
 
#include "CameraFrame.hpp"
 
class ICamera 
{ 
public:

    ~ICamera() = default; 

    virtual bool init() = 0; 
    virtual CameraFrame getFrame() = 0; 

private:
   
};
#endif //ICAMERA_HPP