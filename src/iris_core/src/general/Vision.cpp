
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Vision.h"
#include "ObjectLocatorModule.h"
#include "BoardLocatorModule.h"
#include "ConfigManager.hpp"

Vision::Vision() : mModule(nullptr)
{
    // parse config file for configured object types with an apriltag board 
    YAML::Node tagDetectionConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("tag_detection", tagDetectionConfig))
    {
        throw std::runtime_error("Failed to get sub config for tag_detection"); 
    }

    std::string packagePath = ament_index_cpp::get_package_share_directory("iris_core");
    std::string configPath = packagePath + "/configuration/";

    for(const auto& board : tagDetectionConfig["boards"])
    {
        KnownObjectConfig cfg; 
        cfg.mType = board["type"].as<std::string>(); 
        cfg.mFile = configPath + board["file"].as<std::string>(); 
        mKnownObjects.insert({cfg.mType, cfg});

        LOGV << "Added '" << cfg.mType << "' to known objects set!";   
    }
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

bool Vision::dispatch(const std::string& anObjectTypeToFind)
{
    if(mKnownObjects.find(anObjectTypeToFind) != mKnownObjects.end())
    {
        // pre-known object, locate via apriltags 
        find_tags(mKnownObjects.at(anObjectTypeToFind)); 
    }
    else
    {
        find_object(anObjectTypeToFind); 
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

void Vision::find_tags(const KnownObjectConfig& anObjectToFind)
{
    LOGD << "Commanded to find object with AprilTags!"; 

    if(nullptr != mModule)
    {
        mModule->stop(); 
        return; 
    }

    mModule = std::make_unique<BoardLocatorModule>(anObjectToFind); 
    mModuleThread = std::thread([this]() {
        mModule->start(); 
    }); 
}