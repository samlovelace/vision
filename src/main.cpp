#include <cstdio> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ConfigManager.hpp"
#include "Logger.hpp"
#include "Vision.h"
#include "Calibration.h"
#include <rclcpp/rclcpp.hpp>
#include "CommandHandler.h"

int main()
{
    rclcpp::init(0, nullptr); 
    createLogger(); 
    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/config.yaml";

    ConfigManager::get().load(configPath); 
    LOGV << "############## Configuration: #################\n" << YAML::Dump(ConfigManager::get().getFullConfig()); 

    auto vision = std::make_shared<Vision>(); 

    CommandHandler ch(vision); 
    ch.init(); 
    
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }


    rclcpp::shutdown(); 
}
