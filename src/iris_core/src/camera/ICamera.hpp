#ifndef ICAMERA_HPP
#define ICAMERA_HPP
 
#include "CameraData.hpp"

class ICamera 
{ 
public:

    ~ICamera() = default; 

    virtual bool init() = 0; 
    virtual bool fini() = 0; 
    virtual CameraOutput getOutput() = 0; 

private:
   
};
#endif //ICAMERA_HPP