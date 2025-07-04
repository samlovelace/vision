
#include "PointCloudViewer.h"
#include "plog/Log.h"

PointCloudViewer::PointCloudViewer(const std::string& viewerName)
    : mViewerName(viewerName), mCloudId("cloud"), mRunning(false)
{

}

PointCloudViewer::~PointCloudViewer() 
{
    stop();
}

void PointCloudViewer::start() 
{
    mRunning = true;
    mViewer = std::make_shared<pcl::visualization::PCLVisualizer>(mViewerName);
    mViewer->setBackgroundColor(0, 0, 0);
    mViewer->addCoordinateSystem(0.1);
    mViewer->initCameraParameters();

    //mThread = std::thread(&PointCloudViewer::run, this);
}

void PointCloudViewer::stop() 
{
    mRunning = false;
    if (mThread.joinable()) {
        mThread.join();
    }
}

void PointCloudViewer::updateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr newCloud) 
{
    std::lock_guard<std::mutex> lock(mMutex);
    mCloud = newCloud;
}

void PointCloudViewer::run() 
{
    LOGW << "Cloud viewer run started!"; 
    while (mRunning && !mViewer->wasStopped()) 
    {
        std::lock_guard<std::mutex> lock(mMutex);

        if (mCloud) 
        {
            LOGW << "HAVE A CLOUD TO VISUALIZE"; 
            if (!mViewer->updatePointCloud(mCloud, mCloudId)) 
            {
                mViewer->addPointCloud(mCloud, mCloudId);
                mViewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, mCloudId);
            }
        }
        
        LOGW << "SPIN ONCE"; 
        mViewer->spinOnce(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
