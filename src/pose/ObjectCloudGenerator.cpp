
#include "ObjectCloudGenerator.h"
#include "plog/Log.h"

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

    LOGW << "Bbox x,y,h,w: " << aBoundingBox.x << ", " << aBoundingBox.y << ", " << aBoundingBox.height << ", " << aBoundingBox.width;

    int total = 0, valid = 0;

    auto bbox = aBoundingBox; 
    auto disparity_float = aDepthMap; 

    for (int x = bbox.x; x < bbox.x + bbox.width; ++x) {
        for (int y = bbox.y; y < bbox.y + bbox.height; ++y) {
            float d = disparity_float.at<float>(y, x);
            if (d > 0.0f) ++valid;
            ++total;
        }
    }

    std::cout << "Valid disparity pixels: " << valid << "/" << total << std::endl;

    float baseline = 0.12; // in meters

    for (int x = aBoundingBox.x; x < aBoundingBox.x + aBoundingBox.width; ++x)
    {
        for (int y = aBoundingBox.y; y < aBoundingBox.y + aBoundingBox.height; ++y)
        {
            float disparity = aDepthMap.at<float>(y, x);  
            
            if (disparity <= 0.0f || std::isnan(disparity))
                continue;

            float Z = (fx * baseline) / disparity;

            float X = (x - cx) * Z / fx;
            float Y = (y - cy) * Z / fy;

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

cv::Point3f ObjectCloudGenerator::computeCloudCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloud)
{
    float xSum, ySum, zSum = 0.0; 

    for(const auto& pt : aCloud->points)
    {
        xSum += pt.x; 
        ySum += pt.y; 
        zSum += pt.z; 
    }

    cv::Point3f p(xSum / aCloud->points.size(), ySum / aCloud->points.size(), zSum / aCloud->points.size());
    return p; 
}

void ObjectCloudGenerator::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr anInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloudOut, cv::Matx44f aT_ref_cam)
{
    for (const auto& pt : anInputCloud->points)
    {
        // Convert to homogeneous coordinates
        cv::Vec4f camPt(pt.x, pt.y, pt.z, 1.0f);
        cv::Vec4f transformedPt = aT_ref_cam * camPt;

        pcl::PointXYZ out;
        out.x = transformedPt[0];
        out.y = transformedPt[1];
        out.z = transformedPt[2];
        aCloudOut->points.push_back(out);
    }

    aCloudOut->width = aCloudOut->points.size();
    aCloudOut->height = 1;
    aCloudOut->is_dense = true;
}