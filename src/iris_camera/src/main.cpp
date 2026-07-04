#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include "iris_common/Logger.hpp"
#include "iris_common/RosTopicManager.hpp"
#include "iris_camera/CameraDriver.h"

int main()
{
    rclcpp::init(0, nullptr);
    createLogger();
    RosTopicManager::getInstance("iris_camera");

    std::string packagePath = ament_index_cpp::get_package_share_directory("iris_bringup");
    std::string configPath = packagePath + "/configuration/config.yaml";

    YAML::Node config = YAML::LoadFile(configPath);
    YAML::Node camerasConfig = config["cameras"];

    if(!camerasConfig)
    {
        throw std::runtime_error("Missing camera configuration");
    }

    CameraDriver driver(camerasConfig);
    driver.run();

    RosTopicManager::getInstance()->spinNode();

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
}
