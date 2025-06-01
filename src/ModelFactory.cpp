
#include "ModelFactory.h"
#include "YoloModel.h"
#include "MobileNetSSDModel.hpp"

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
    else{
        LOGE << "Unsupported model type: " << aModelType; 
        return nullptr; 
    }
}