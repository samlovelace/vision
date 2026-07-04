#include <cstdio> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ConfigManager.hpp"
#include "iris_common/Logger.hpp"
#include "Vision.h"
#include "Calibration.h"
#include <rclcpp/rclcpp.hpp>
#include "CommandHandler.h"
#include "iris_common/RosTopicManager.hpp"

int main()
{
    rclcpp::init(0, nullptr);
    createLogger();
    RosTopicManager::getInstance("iris_core");
    std::string packagePath = ament_index_cpp::get_package_share_directory("iris_bringup");
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
