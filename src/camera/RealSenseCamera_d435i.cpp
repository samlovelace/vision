
#include "RealSenseCamera_d435i.h"
#include "plog/Log.h"

RealSenseCamera_d435i::RealSenseCamera_d435i()
{

}

RealSenseCamera_d435i::~RealSenseCamera_d435i()
{
    if(mProcessThread.joinable())
    {
        mProcessThread.join(); 
    }
}

bool RealSenseCamera_d435i::init()
{
    mPipeline = std::make_unique<rs2::pipeline>(); 
    
    if(nullptr == mPipeline)
    {
        LOGE << "Failed to initialize RealSenseCamera_d435i"; 
        return false;  
    }

    mPipeline->start();
    mStartTime = std::chrono::system_clock::now(); 
    
    for(int i = 0; i < 30; i++)
    {
        // skip initial frames for auto exposure
        mPipeline->wait_for_frames(); 
    }

    mRunning = true;
    mProcessThread = std::thread(&RealSenseCamera_d435i::processFrames, this); 
    return true; 
}

CameraOutput RealSenseCamera_d435i::getOutput()
{
    std::lock_guard<std::mutex> lock(mFrameMutex); 
    return mLatest; 
}

void RealSenseCamera_d435i::processFrames()
{
    while (mRunning)
    {
        rs2::frameset frames = mPipeline->wait_for_frames();
        
        rs2::depth_frame rawDepth = frames.get_depth_frame();
        rs2::video_frame rgb = frames.get_color_frame();

        int width = rgb.get_width();
        int height = rgb.get_height();

        cv::Mat colorImageBGR(cv::Size(width, height), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat colorImageRGB;
        cv::cvtColor(colorImageBGR, colorImageRGB, cv::COLOR_BGR2RGB);

        int depthWidth = rawDepth.get_width();
        int depthHeight = rawDepth.get_height();

        cv::Mat raw_depth(depthHeight, depthWidth, CV_16UC1, (void*)rawDepth.get_data(), cv::Mat::AUTO_STEP);

        auto sensor = mPipeline->get_active_profile().get_device().first<rs2::depth_sensor>();
        float scale = sensor.get_depth_scale();

        cv::Mat depth_m;
        raw_depth.convertTo(depth_m, CV_32FC1, scale);

        auto timestamp = mStartTime + std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double, std::milli>(frames.get_timestamp()));

        CameraFrame left;
        left.mFrame = colorImageRGB;
        left.mTimestamp = timestamp;

        CameraFrame depth;
        depth.mFrame = depth_m;
        depth.mTimestamp = timestamp;

        CameraOutput output;
        output.depth = depth;
        output.left = left;

        output.isDepth = true;
        output.isStereo = false;

        {
            std::lock_guard<std::mutex> lock(mFrameMutex);
            mLatest = output;
        }
    }
}