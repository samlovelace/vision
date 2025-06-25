#ifndef VISION_H
#define VISION_H
 
#include "IModule.hpp"
#include <memory> 
#include <thread> 
 
class Vision 
{ 
public:
    Vision();
    ~Vision();

    void find_object(const std::string& anObjTypeToFind); 
    void stop(); 

private:

    std::unique_ptr<IModule> mModule; 
    std::thread mModuleThread; 
   
};
#endif //VISION_H