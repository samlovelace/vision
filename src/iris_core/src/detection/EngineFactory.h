#ifndef ENGINEFACTORY_H
#define ENGINEFACTORY_H
 
#include "IInferenceEngine.hpp" 
#include <string> 
#include <memory>

class EngineFactory 
{ 
public:
    EngineFactory() {}
    ~EngineFactory() {}

    static std::unique_ptr<IInferenceEngine> create(const std::string& anEngineType); 

private:
   
};
#endif //ENGINEFACTORY_H