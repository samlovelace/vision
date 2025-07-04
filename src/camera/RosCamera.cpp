
#include "RosCamera.h"
#include "RosTopicManager.hpp"
#include <cv_bridge/cv_bridge.h>

RosCamera::RosCamera()
{

}

RosCamera::~RosCamera()
{

}

bool RosCamera::init()
{
    using namespace message_filters;

    auto node = RosTopicManager::getInstance();

    // Create message_filters subscribers instead of plain ROS subscriptions
    mLeftFrameSub.subscribe(node, "/stereo/left/image_raw");
    mRightFrameSub.subscribe(node, "/stereo/right/image_raw");

    // ApproximateTime policy allows for small timestamp differences
    mSync = std::make_shared<Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(10));

    mSync->connectInput(mLeftFrameSub, mRightFrameSub);

    mSync->registerCallback(
      std::bind(&RosCamera::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

    while(mOutput.left.mFrame.empty())
    {
        // wait until first frame arrives
    }

    return true; 
}

CameraOutput RosCamera::getOutput()
{
    std::lock_guard<std::mutex> lock(mOutputMutex); 
    return mOutput; 
}

void RosCamera::stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr aLeftImg, const sensor_msgs::msg::Image::ConstSharedPtr aRightImg)
{
    cv::Mat left; 
    cv::Mat right; 

    left = cv_bridge::toCvCopy(aLeftImg, "rgb8")->image;
    right = cv_bridge::toCvCopy(aRightImg, "rgb8")->image; 

    auto leftTime = rosTimeToChrono(aLeftImg->header.stamp);
    auto rightTime = rosTimeToChrono(aRightImg->header.stamp); 

    CameraFrame rightFrame(right, rightTime); 
    CameraFrame leftFrame(left, leftTime); 

    CameraOutput output(leftFrame, rightFrame, true); 

    // update the output
    {
        std::lock_guard<std::mutex> lock(mOutputMutex);
        mOutput = output; 
    }
} 

std::chrono::system_clock::time_point RosCamera::rosTimeToChrono(const builtin_interfaces::msg::Time& stamp)
{
    auto total_nanos =
        std::chrono::seconds(stamp.sec) +
        std::chrono::nanoseconds(stamp.nanosec);

    return std::chrono::system_clock::time_point(total_nanos);
}
