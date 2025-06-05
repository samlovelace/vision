#include <cstdio> 
#include "ConfigManager.hpp"
#include "Logger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Vision.h"
#include "Calibration.h"

int main()
{
    createLogger(); 
    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/config.yaml";

    //TODO: dump full config file in log 
    ConfigManager::get().load(configPath); 

    // TODO: put this chunk somehwere else? 
    std::string mode; 
    if(!ConfigManager::get().getConfig<std::string>("mode", mode))
    {
        throw std::invalid_argument("invalid mode configuration"); 
    }

    std::unique_ptr<IModule> perception = nullptr; 

    if("vision" == mode)
    {
        perception = std::make_unique<Vision>(); 
    }
    else if ("calibration" == mode)
    {
        perception = std::make_unique<Calibration>(); 
    }
    else
    {
        LOGE << "Unsupported mode: " << mode; 
        return 0; 
    }

    perception->start(); 
}
