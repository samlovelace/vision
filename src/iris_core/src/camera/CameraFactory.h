#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H
 
#include "ICamera.hpp"
#include <yaml-cpp/yaml.h>
 
class CameraFactory 
{ 
public:
    CameraFactory();
    ~CameraFactory();

    static std::shared_ptr<ICamera> create(const std::string& aCamType); 

private:
   
};
#endif //CAMERAFACTORY_H