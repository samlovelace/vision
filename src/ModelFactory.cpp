
#include "ModelFactory.h"
#include "YoloModel.h"

#include "plog/Log.h"

std::unique_ptr<IModel> ModelFactory::create(const std::string& aModelType)
{
    if("yolo" == aModelType)
    {
        return std::make_unique<YoloModel>(); 
    }
    else{
        LOGE << "Unsupported model type: " << aModelType; 
        return nullptr; 
    }
}