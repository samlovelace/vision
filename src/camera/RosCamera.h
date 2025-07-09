#ifndef ROSCAMERA_H
#define ROSCAMERA_H
 
#include "ICamera.hpp"
#include <mutex> 

#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 

class RosCamera : public ICamera
{ 

public:
    RosCamera();
    ~RosCamera();

    bool init() override; 
    CameraOutput getOutput() override; 

private:

    CameraOutput mOutput; 
    std::mutex mOutputMutex; 

    std::chrono::system_clock::duration mTimeOffset; 
    bool mOffsetComputed; 

    using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image>;

    message_filters::Subscriber<sensor_msgs::msg::Image> mLeftFrameSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> mRightFrameSub;

    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> mSync;

    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr aLeftImg, const sensor_msgs::msg::Image::ConstSharedPtr aRightImg);
    std::chrono::system_clock::time_point rosTimeToChrono(const builtin_interfaces::msg::Time& stamp);
    
};
#endif //ROSCAMERA_H