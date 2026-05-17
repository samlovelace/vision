
#include "EngineFactory.h"
#include "OpenCvEngine.h"

#include "plog/Log.h"

std::unique_ptr<IInferenceEngine> EngineFactory::create(const std::string& anEngineType)
{
    if("opencv" == anEngineType)
    {
        return std::make_unique<OpenCvEngine>(); 
    }
    else{
        LOGE << "Unsupported inference engine type: " << anEngineType; 
        return nullptr; 
    }
}