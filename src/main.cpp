#include <cstdio> 
#include "ConfigManager.hpp"
#include "Logger.hpp"
#include "ModelHandler.h"
#include "CameraHandler.h"
#include <ament_index_cpp/get_package_share_directory.hpp>


int main()
{
    createLogger(); 
    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/config.yaml";

    ConfigManager::get().load(configPath); 

    auto mh = std::make_shared<ModelHandler>(); 
    auto ch = std::make_shared<CameraHandler>(mh); 

    YAML::Node modelsConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("models", modelsConfig))
    {
        LOGE << "Unable to get models config"; 
        return 0; 
    }

    mh->setupModels(modelsConfig);
    mh->test(); 

    YAML::Node cameraConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("camera", cameraConfig))
    {
        LOGE << "Invalid camera config"; 
        return 0; 
    }

    ch->init(cameraConfig); 
    ch->run(); 
}
