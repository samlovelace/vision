#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H
 
#include <yaml-cpp/yaml.h>
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
#include "IDepthEstimator.hpp"
#include "InferenceHandler.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

class PoseEstimationHandler 
{ 
public:
    PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                          std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                          std::shared_ptr<InferenceHandler> anInferenceHandler, 
                          std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue, 
                          std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> aPcQueue); 
    ~PoseEstimationHandler();

    void run(); 

private:
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mDepthMapVisQueue; 
    std::shared_ptr<IDepthEstimator> mDepthEstimator; 
    std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> mCloudVisQueue; 
    bool mVisualizationLaunched;

    pcl::visualization::PCLVisualizer::Ptr mViewer; 
    std::mutex mViewerMutex;        

    void visualizeNonBlocking(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

};
#endif //POSEESTIMATOR_H