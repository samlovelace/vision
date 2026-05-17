#ifndef CALIBRATION_H
#define CALIBRATION_H
 
#include "IModule.hpp"
 
class Calibration : public IModule
{ 
public:
    Calibration();
    ~Calibration();

    void start() override; 

private:
   
};
#endif //CALIBRATION_H