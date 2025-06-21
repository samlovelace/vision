#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H
 
#include <yaml-cpp/yaml.h>
#include "ConcurrentQueue.hpp"
#include "Detection.hpp"
#include "IDepthEstimator.hpp"
#include "InferenceHandler.h"
#include "ObjectCloudGenerator.h"
#include "DetectedObjectManager.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// TODO: maybe this deserves a better name. Its dealing with computing the object point cloud 
// and global centroid and passing to the DetectedObjectManager 
class PoseEstimationHandler 
{ 
public:
    PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                          std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                          std::shared_ptr<InferenceHandler> anInferenceHandler, 
                          std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue, 
                          std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>> aPcQueue, 
                          std::shared_ptr<DetectedObjectManager> anObjManager); 
    ~PoseEstimationHandler();

    void run(); 

private:
    std::shared_ptr<ConcurrentQueue<Detection>> mDetectionQueue; 
    std::shared_ptr<ConcurrentQueue<cv::Mat>> mDepthMapVisQueue; 
    std::shared_ptr<IDepthEstimator> mDepthEstimator; 
    std::shared_ptr<ObjectCloudGenerator> mObjCloudGenerator; 
    std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>> mCloudVisQueue; 
    std::shared_ptr<DetectedObjectManager> mDetectedObjectManager; 

};
#endif //POSEESTIMATOR_H