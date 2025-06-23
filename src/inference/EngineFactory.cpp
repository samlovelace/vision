
#include "EngineFactory.h"
#include "OpenCvEngine.h"
#include "OnnxEngine.h"

#include "plog/Log.h"

std::unique_ptr<IInferenceEngine> EngineFactory::create(const std::string& anEngineType)
{
    if("opencv" == anEngineType)
    {
        return std::make_unique<OpenCvEngine>(); 
    }
    else if("onnx" == anEngineType)
    {
        return std::make_unique<OnnxEngine>(); 
    }
    else{
        LOGE << "Unsupported inference engine type: " << anEngineType; 
        return nullptr; 
    }
}