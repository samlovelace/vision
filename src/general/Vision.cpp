
#include "Vision.h"
#include "ObjectLocatorModule.h"
#include "BoardLocatorModule.h"

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
        
        mModule = nullptr; 
    }
}

void Vision::find_object(const std::string& anObjectTypeToFind)
{
    LOGD << "Commanded to find object of type: " << anObjectTypeToFind; 
    
    if(nullptr != mModule)
    {
        // TODO: handle shutdown of current module properly
        mModule->stop(); 
        return; 
    }

    mModule = std::make_unique<ObjectLocatorModule>(anObjectTypeToFind);

    mModuleThread = std::thread([this](){
        mModule->start(); 
    });
}

void Vision::find_tags(const std::string& aBoardToFind)
{
    LOGD << "Commanded to find object with AprilTags!"; 

    if(nullptr != mModule)
    {
        mModule->stop(); 
        return; 
    }

    mModule = std::make_unique<BoardLocatorModule>(aBoardToFind); 
    mModuleThread = std::thread([this]() {
        mModule->start(); 
    }); 
}