#include <cstdio> 
#include "ConfigManager.hpp"
#include "Logger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


int main()
{
    createLogger(); 
    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/config.yaml";

    ConfigManager::get().load(configPath); 

}
