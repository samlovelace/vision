#ifndef NAVDATAHANDLER_H
#define NAVDATAHANDLER_H
  
#include "nora_idl/msg/robot_state.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/opencv.hpp>


class NavDataHandler 
{ 
public:
    NavDataHandler(const std::string& aNavTopicName);
    ~NavDataHandler();

    cv::Matx44f getMatchingGlobalPose(const std::chrono::system_clock::time_point& aTime);

private:

    std::map<std::chrono::system_clock::time_point, cv::Matx44f> mGlobalPoseMap; 
    void onDataRecvd(nora_idl::msg::RobotState::SharedPtr aRobotState);
    cv::Matx44f transformFromXYZQuat(float x, float y, float z, float qx, float qy, float qz, float qw);
   
};
#endif //NAVDATAHANDLER_H