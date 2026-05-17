
#include "ModelFactory.h"
#include "YoloModel.h"
#include "MobileNetSSDModel.hpp"
#include "MidasModel.h"

#include "plog/Log.h"

std::unique_ptr<IModel> ModelFactory::create(const std::string& aModelType)
{
    if("yolo" == aModelType)
    {
        return std::make_unique<YoloModel>(); 
    }
    else if ("mobilenet" == aModelType)
    {
        return std::make_unique<MobileNetSSDModel>(); 
    }
    else if ("midas" == aModelType)
    {
        return std::make_unique<MidasModel>(); 
    }
    else{
        LOGE << "Unsupported model type: " << aModelType; 
        return nullptr; 
    }
}