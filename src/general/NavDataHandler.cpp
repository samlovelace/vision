
#include "NavDataHandler.h"
#include "RosTopicManager.hpp"

NavDataHandler::NavDataHandler(const std::string& aNavTopicName)
{
    RosTopicManager::getInstance()->createSubscriber<nora_idl::msg::RobotState>(aNavTopicName, 
                                                                                std::bind(&NavDataHandler::onDataRecvd, 
                                                                                          this, 
                                                                                          std::placeholders::_1)); 

}

NavDataHandler::~NavDataHandler()
{

}

void NavDataHandler::onDataRecvd(nora_idl::msg::RobotState::SharedPtr aRobotState)
{
    using namespace std::chrono; 
    // process nav data and push to 
    builtin_interfaces::msg::Time timestamp = aRobotState->timestamp;
    
    system_clock::time_point time(seconds(timestamp.sec) + nanoseconds(timestamp.nanosec));

    nora_idl::msg::Vec3 pos = aRobotState->position; 
    nora_idl::msg::Quaternion q = aRobotState->quat; 

    cv::Matx44f T_G_C = transformFromXYZQuat(pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w); 

    mGlobalPoseMap.insert({time, T_G_C}); 
}

// Create a 4x4 transform matrix from quaternion + translation
cv::Matx44f NavDataHandler::transformFromXYZQuat(float x, float y, float z, float qx, float qy, float qz, float qw)
{
    // Normalize the quaternion just in case
    float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;

    // Convert quaternion to rotation matrix
    float xx = qx * qx;
    float yy = qy * qy;
    float zz = qz * qz;
    float xy = qx * qy;
    float xz = qx * qz;
    float yz = qy * qz;
    float wx = qw * qx;
    float wy = qw * qy;
    float wz = qw * qz;

    cv::Matx44f mat = cv::Matx44f::eye();

    mat(0,0) = 1.0f - 2.0f * (yy + zz);
    mat(0,1) = 2.0f * (xy - wz);
    mat(0,2) = 2.0f * (xz + wy);
    mat(0,3) = x;

    mat(1,0) = 2.0f * (xy + wz);
    mat(1,1) = 1.0f - 2.0f * (xx + zz);
    mat(1,2) = 2.0f * (yz - wx);
    mat(1,3) = y;

    mat(2,0) = 2.0f * (xz - wy);
    mat(2,1) = 2.0f * (yz + wx);
    mat(2,2) = 1.0f - 2.0f * (xx + yy);
    mat(2,3) = z;

    return mat;
}

cv::Matx44f NavDataHandler::getMatchingGlobalPose(const std::chrono::system_clock::time_point& aTime)
{
    if(mGlobalPoseMap.empty())
    { 
        return cv::Matx44f::eye();  
    }

    auto it = mGlobalPoseMap.lower_bound(aTime); 

    // only entry in map 
    if(it == mGlobalPoseMap.begin())
    {
        return it->second; 
    }

    if(it == mGlobalPoseMap.end())
    {
        return std::prev(it)->second; 
    }

    // Compare it and the previous timestamp to see which is closer
    auto after = it;
    auto before = std::prev(it);

    auto diff_after = std::chrono::duration_cast<std::chrono::milliseconds>(after->first - aTime).count();
    auto diff_before = std::chrono::duration_cast<std::chrono::milliseconds>(aTime - before->first).count();

    return (diff_before <= diff_after) ? before->second : after->second;
}