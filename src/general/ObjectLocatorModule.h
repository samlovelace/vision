#ifndef OBJECTLOCATORMODULE_H
#define OBJECTLOCATORMODULE_H
 
#include "CameraHandler.h"
#include "ConcurrentQueue.hpp" 
#include "ObjectDetectionHandler.h"
#include "PoseEstimationHandler.h"
#include "InferenceHandler.h"
#include "IModule.hpp"
#include "CameraData.hpp"
#include "NavDataHandler.h"
#include "DetectedObjectManager.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

#include <thread> 

class ObjectLocatorModule : public IModule
{ 
public:
    ObjectLocatorModule(const std::string& anObjectType);
    ~ObjectLocatorModule();

    void start() override; 
    void stop() override; 

    bool isRunning() {std::lock_guard<std::mutex> lock(mRunningMutex); return mRunning; }
    void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunningMutex); mRunning = aFlag; }

private:

    std::shared_ptr<InferenceHandler> mInferenceHandler; 

    std::vector<std::thread> mThreads; 

    std::shared_ptr<CameraHandler> mCameraHandler; 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue; 

    std::shared_ptr<ObjectDetectionHandler> mDetector; 
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<Detection>> m2DVisQueue; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mDepthFrameVisQueue; 

    std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>> mCloudVisQueue; 
    std::shared_ptr<PoseEstimationHandler> mPoseEstimationHandler;
    std::shared_ptr<DetectedObjectManager> mObjectManager; 

    std::shared_ptr<NavDataHandler> mNavDataHandler; 

    bool mVisualize; 
    bool mVisualizePointCloud;
    bool mSavePointCloud; 
    bool mRunning; 
    std::mutex mRunningMutex; 

    pcl::visualization::PCLVisualizer::Ptr mViewer;
    bool mViewerInitialized = false;
    std::string mObjectToLocate; 

    void runVisualizer();

   
};
#endif //OBJECTLOCATORMODULE_H