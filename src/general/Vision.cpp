
#include "Vision.h"
#include "ObjectLocatorModule.h"

Vision::Vision() : mModule(nullptr)
{

}

Vision::~Vision()
{

}

void Vision::find_object(const std::string& anObjectTypeToFind, const std::string& aCentroidMethod)
{
    if(nullptr != mModule)
    {
        // TODO: handle shutdown of current module properly
    }

    mModule = std::make_unique<ObjectLocatorModule>(anObjectTypeToFind, aCentroidMethod);

    mModuleThread = std::thread([this](){
        mModule->start(); 
    });
}