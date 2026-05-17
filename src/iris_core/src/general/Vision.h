#ifndef VISION_H
#define VISION_H

#include <unordered_map> 
#include <memory> 
#include <thread> 

#include "IModule.hpp"
#include "Types.hpp"

class Vision 
{ 
public:
    Vision();
    ~Vision();

    bool dispatch(const std::string& anObjectTypeToFind);
    void stop();

private: 

    void find_object(const std::string& anObjTypeToFind); 
    void find_tags(const KnownObjectConfig& anObjectToFind); 

private:

    std::unique_ptr<IModule> mModule; 
    std::thread mModuleThread; 

    std::unordered_map<std::string, KnownObjectConfig> mKnownObjects; 
   
};
#endif //VISION_H