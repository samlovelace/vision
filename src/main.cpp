#include <cstdio> 
#include "ConfigManager.hpp"
#include "Logger.hpp"
#include "ModelHandler.h"
#include "CameraHandler.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Vision.h"

int main()
{
    createLogger(); 
    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/config.yaml";

    ConfigManager::get().load(configPath); 

    Vision vision; 
    vision.start(); 

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
}
