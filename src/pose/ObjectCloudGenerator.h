#ifndef OBJECTCLOUDGENERATOR
#define OBJECTCLOUDGENERATOR
 
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "CameraParams.hpp"

class ObjectCloudGenerator 
{ 
public:
    ObjectCloudGenerator();
    ~ObjectCloudGenerator();

    pcl::PointCloud<pcl::PointXYZ>::Ptr generateCloud(const cv::Rect& aBoundingBox, const cv::Mat& aDepthMap, std::shared_ptr<CameraParams> aCamParams);
    cv::Point3f computeCloudCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloud);
    void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr anInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloudOut, cv::Matx44f aT_ref_cam); 

private:
   
};
#endif //OBJECTCLOUDGENERATOR