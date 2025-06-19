
#include "ObjectCloudGenerator.h"

ObjectCloudGenerator::ObjectCloudGenerator()
{

}

ObjectCloudGenerator::~ObjectCloudGenerator()
{

}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectCloudGenerator::generateCloud(const cv::Rect& aBoundingBox, const cv::Mat& aDepthMap, std::shared_ptr<CameraParams> aCamParams)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // save camera intrinsics in local variable for easy use
    float fx = aCamParams->mIntrinsics->focalX(); 
    float fy = aCamParams->mIntrinsics->focalY();
    float cx = aCamParams->mIntrinsics->centerX();
    float cy = aCamParams->mIntrinsics->centerY(); 

    for(int x = aBoundingBox.x; x < aBoundingBox.height; x++)
    {
        for(int y = aBoundingBox.y; y < aBoundingBox.width; y++)
        {
            float d = aDepthMap.at<float>(y, x); 
            
            if(d <= 0.0f || std::isnan(d))
            {
                continue; 
            }

            float X = (x - cx) * d/fx; 
            float Y = (y - cy) * d/fy; 
            float Z = d; 
            
            pcl::PointXYZ pt;
            pt.x = X;
            pt.y = Y;
            pt.z = Z;

            cloud->points.push_back(pt);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud; 
}