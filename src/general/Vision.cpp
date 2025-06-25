
#include "Vision.h"
#include "ObjectLocatorModule.h"

Vision::Vision() : mModule(nullptr)
{

}

Vision::~Vision()
{

}

void Vision::stop()
{
    if(nullptr != mModule)
    {
        mModule->stop(); 

        if(mModuleThread.joinable())
        {
            mModuleThread.join(); 
        }
    }
}

void Vision::find_object(const std::string& anObjectTypeToFind)
{
    if(nullptr != mModule)
    {
        // TODO: handle shutdown of current module properly
    }

    mModule = std::make_unique<ObjectLocatorModule>(anObjectTypeToFind);

    mModuleThread = std::thread([this](){
        mModule->start(); 
    });
}