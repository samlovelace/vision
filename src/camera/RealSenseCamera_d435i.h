#ifndef REALSENSECAMERA_D435i_H
#define REALSENSECAMERA_D435i_H
 
#include "ICamera.hpp"

#include "librealsense2/rs.hpp"
#include <thread>
 
class RealSenseCamera_d435i : public ICamera
{ 
public:
    RealSenseCamera_d435i();
    ~RealSenseCamera_d435i();

    bool init() override; 
    CameraOutput getOutput() override; 

private:

    void processFrames(); 
    void setLatestFrames(const cv::Mat& anRgbFrame, const cv::Mat& aDepthFrame); 
    std::unique_ptr<rs2::pipeline> mPipeline; 

    std::thread mProcessThread; 
    std::mutex mFrameMutex; 
    CameraOutput mLatest; 

    std::chrono::system_clock::time_point mStartTime; 
   
};
#endif //REALSENSECAMERA_D435i_H