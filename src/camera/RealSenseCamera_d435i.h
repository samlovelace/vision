#ifndef REALSENSECAMERA_D435i_H
#define REALSENSECAMERA_D435i_H
 
#include "ICamera.hpp"
 
class RealSenseCamera_d435i : public ICamera
{ 
public:
    RealSenseCamera_d435i();
    ~RealSenseCamera_d435i();

    bool init() override; 
    CameraOutput getOutput() override; 

private:
   
};
#endif //REALSENSECAMERA_D435i_H