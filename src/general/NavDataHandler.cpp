
#include "NavDataHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"

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

    builtin_interfaces::msg::Time timestamp = aRobotState->timestamp;

    auto duration_since_epoch =
        seconds(timestamp.sec) + nanoseconds(timestamp.nanosec);

    system_clock::time_point time_point(duration_since_epoch);

    nora_idl::msg::Vec3 pos = aRobotState->position;
    nora_idl::msg::Quaternion q = aRobotState->quat;

    cv::Matx44f T_G_V = transformFromXYZQuat(pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w);

    mGlobalPoseMap.insert({time_point, T_G_V});
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

cv::Matx44f NavDataHandler::getClosestGlobalPose(
    const std::chrono::system_clock::time_point& aTime)
{
    if (mGlobalPoseMap.empty())
    {
        LOGD << "Nav data map is empty";
        return cv::Matx44f::eye();
    }

    auto cam_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  aTime.time_since_epoch()).count();

    LOGD << "CAMERA timestamp (ms): " << cam_ms;

    auto it = mGlobalPoseMap.lower_bound(aTime);

    if (it == mGlobalPoseMap.begin())
    {
        LOGW << "Queried time earlier than first nav data timestamp.";
        return it->second;
    }

    if (it == mGlobalPoseMap.end())
    {
        LOGW << "Queried time later than last nav data timestamp.";
        return std::prev(it)->second;
    }

    auto after = it;
    auto before = std::prev(it);

    auto diff_after = std::chrono::abs(after->first - aTime);
    auto diff_before = std::chrono::abs(before->first - aTime);

    LOGD << "Nav data map size: " << mGlobalPoseMap.size();
    LOGD << "Query time (ms): "
         << std::chrono::duration_cast<std::chrono::milliseconds>(aTime.time_since_epoch()).count();
    LOGD << "Before time (ms): "
         << std::chrono::duration_cast<std::chrono::milliseconds>(before->first.time_since_epoch()).count();
    LOGD << "After time (ms): "
         << std::chrono::duration_cast<std::chrono::milliseconds>(after->first.time_since_epoch()).count();

    LOGD << "diff_before (ms): "
         << std::chrono::duration_cast<std::chrono::milliseconds>(diff_before).count();
    LOGD << "diff_after (ms): "
         << std::chrono::duration_cast<std::chrono::milliseconds>(diff_after).count();

    auto match = (diff_before <= diff_after) ? before : after;
    
    mGlobalPoseMap.erase(mGlobalPoseMap.begin(), it); 
    return match->second; 
}
