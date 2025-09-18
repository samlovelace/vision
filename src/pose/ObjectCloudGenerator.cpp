
#include "ObjectCloudGenerator.h"
#include "plog/Log.h"

ObjectCloudGenerator::ObjectCloudGenerator()
{

}

ObjectCloudGenerator::~ObjectCloudGenerator()
{

}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectCloudGenerator::generateCloud_openCV(const cv::Rect& aBoundingBox, const cv::Mat& aDepthMap, std::shared_ptr<CameraParams> aCamParams)
{
    // save camera intrinsics in local variable for easy use
    float fx = aCamParams->mIntrinsics->focalX(); 
    float fy = aCamParams->mIntrinsics->focalY();
    float cx = aCamParams->mIntrinsics->centerX();
    float cy = aCamParams->mIntrinsics->centerY();
    float near = aCamParams->mIntrinsics->nearPlane_m; 
    float far = aCamParams->mIntrinsics->farPlane_m;  
    float baseline = aCamParams->mIntrinsics->baseline_m; 

    cv::Mat Q = (cv::Mat_<double>(4,4) <<
        1, 0, 0, -cx,
        0, 1, 0, -cy,
        0, 0, 0, fx,
        0, 0, -1.0/baseline, 0);

    // Reproject
    cv::Mat xyz;
    cv::reprojectImageTo3D(aDepthMap, xyz, Q, true);

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int y = 0; y < xyz.rows; ++y)
    {
        for (int x = 0; x < xyz.cols; ++x)
        {
            cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
            float X = point[0];
            float Y = point[1];
            float Z = point[2];

            if (Z <= 0 || std::isinf(Z) || std::isnan(Z)) continue;

            cloud->points.emplace_back(X, Y, Z);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectCloudGenerator::generateCloud(
    const cv::Rect& aBoundingBox,
    const cv::Mat& aDepthMap,
    std::shared_ptr<CameraParams> aCamParams)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //Validate inputs
    if (aDepthMap.empty()) return cloud;
    if (aDepthMap.type() != CV_32FC1) 
    {
        // Convert if you prefer, or just bail:
        // cv::Mat tmp; aDepthMap.convertTo(tmp, CV_32FC1);
        return cloud;
    }
    if (!aCamParams || !aCamParams->mIntrinsics) return cloud;

    // Clamp bbox to image bounds to avoid out-of-range access
    cv::Rect imgRect(0, 0, aDepthMap.cols, aDepthMap.rows);
    cv::Rect roi = aBoundingBox & imgRect;
    if (roi.empty()) return cloud;

    // Pull intrinsics
    float fx = aCamParams->mIntrinsics->focalX();
    float fy = aCamParams->mIntrinsics->focalY();
    float cx = aCamParams->mIntrinsics->centerX();
    float cy = aCamParams->mIntrinsics->centerY();
    float near = aCamParams->mIntrinsics->nearPlane_m;
    float far  = aCamParams->mIntrinsics->farPlane_m;

    // Iterate safely (y outer for cache locality)
    for (int y = roi.y; y < roi.y + roi.height; ++y)
    {
        const float* drow = aDepthMap.ptr<float>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x)
        {
            float Z = drow[x];
            if (!(Z > 0.0f) || std::isnan(Z)) continue; // reject 0, negatives, NaNs
            if (Z < near || Z > far) continue;

            float X = (x - cx) * (Z / fx);
            float Y = (y - cy) * (Z / fy);

            cloud->points.emplace_back(X, Y, Z);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
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