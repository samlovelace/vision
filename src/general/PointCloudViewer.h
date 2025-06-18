#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

class PointCloudViewer {
public:
    PointCloudViewer(const std::string& viewerName = "PointCloud Viewer");
    ~PointCloudViewer();

    void start();
    void stop();
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr newCloud);

private:
    void run();

    std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    std::thread mThread;
    std::mutex mMutex;
    std::atomic<bool> mRunning;

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mCloud;
    std::string mCloudId;
    std::string mViewerName;
};
