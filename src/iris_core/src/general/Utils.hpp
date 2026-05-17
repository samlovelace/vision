#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <iostream>

namespace Utils
{
    inline cv::Matx44f transformFromXYZQuat(std::vector<float> xyz, std::vector<float> quat)
    {
        float x = xyz[0]; 
        float y = xyz[1]; 
        float z = xyz[2]; 

        float qx = quat[0]; 
        float qy = quat[1]; 
        float qz = quat[2]; 
        float qw = quat[3]; 

        // Normalize the quaternion just in case
        float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

        // Convert quaternion to rotation matrix
        float xx = qx * qx;
        float yy = qy * qy;
        float zz = qz * qz;
        float xy = qx * qy;
        float xz = qx * qz;
        float yz = qy * qz;
        float wx = qw * qx;
        float wy = qw * qy;
        float wz = qw * qz;

        cv::Matx44f mat = cv::Matx44f::eye();

        mat(0,0) = 1.0f - 2.0f * (yy + zz);
        mat(0,1) = 2.0f * (xy - wz);
        mat(0,2) = 2.0f * (xz + wy);
        mat(0,3) = x;

        mat(1,0) = 2.0f * (xy + wz);
        mat(1,1) = 1.0f - 2.0f * (xx + zz);
        mat(1,2) = 2.0f * (yz - wx);
        mat(1,3) = y;

        mat(2,0) = 2.0f * (xz - wy);
        mat(2,1) = 2.0f * (yz + wx);
        mat(2,2) = 1.0f - 2.0f * (xx + yy);
        mat(2,3) = z;

        return mat;
    }

    inline float computeIoU(const cv::Rect& a, const cv::Rect& b) 
    {
        cv::Rect inter = a & b;
        float interArea = static_cast<float>(inter.area());
        float unionArea = static_cast<float>(a.area() + b.area() - interArea);
        return (unionArea > 0.0f) ? (interArea / unionArea) : 0.0f;
    }

    template<typename PointT>
    bool savePointCloudAsPLY(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const std::string& filename)
    {
        if (!cloud || cloud->empty())
        {
            //std::cerr << "[savePointCloudAsPLY] Error: Empty or null cloud\n";
            return false;
        }

        if (pcl::io::savePLYFileASCII(filename, *cloud) != 0)
        {
            //std::cerr << "[savePointCloudAsPLY] Failed to save to " << filename << "\n";
            return false;
        }

        //std::cout << "[savePointCloudAsPLY] Saved " << cloud->size() << " points to " << filename << "\n";
        return true;
    }

    inline void printXYZandRPY(const cv::Matx44f& T, const std::string& aName)
    {
        // Extract translation
        float x = T(0, 3);
        float y = T(1, 3);
        float z = T(2, 3);

        // Extract rotation matrix
        cv::Matx33f R(
            T(0,0), T(0,1), T(0,2),
            T(1,0), T(1,1), T(1,2),
            T(2,0), T(2,1), T(2,2)
        );

        // Compute roll, pitch, yaw (XYZ convention)
        float pitch = std::atan2(-R(2,0), std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
        float roll  = std::atan2(R(2,1), R(2,2));
        float yaw   = std::atan2(R(1,0), R(0,0));

        LOGV << aName; 
        LOGV << "XYZ (m): " << x << ", " << y << ", " << z;
        LOGV << "RPY (deg): " << roll * 180.0/M_PI << ", " << pitch * 180.0/M_PI << ", " << yaw * 180.0/M_PI;
    }

    inline cv::Vec4f quatFromMatx44f(const cv::Matx44f& T)
    {
        // Extract rotation elements
        float r00 = T(0,0), r01 = T(0,1), r02 = T(0,2);
        float r10 = T(1,0), r11 = T(1,1), r12 = T(1,2);
        float r20 = T(2,0), r21 = T(2,1), r22 = T(2,2);

        float trace = r00 + r11 + r22;
        float qw, qx, qy, qz;

        if (trace > 0.0f) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            qw = 0.25f / s;
            qx = (r21 - r12) * s;
            qy = (r02 - r20) * s;
            qz = (r10 - r01) * s;
        } else if (r00 > r11 && r00 > r22) {
            float s = 2.0f * std::sqrt(1.0f + r00 - r11 - r22);
            qw = (r21 - r12) / s;
            qx = 0.25f * s;
            qy = (r01 + r10) / s;
            qz = (r02 + r20) / s;
        } else if (r11 > r22) {
            float s = 2.0f * std::sqrt(1.0f + r11 - r00 - r22);
            qw = (r02 - r20) / s;
            qx = (r01 + r10) / s;
            qy = 0.25f * s;
            qz = (r12 + r21) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + r22 - r00 - r11);
            qw = (r10 - r01) / s;
            qx = (r02 + r20) / s;
            qy = (r12 + r21) / s;
            qz = 0.25f * s;
        }

        // Vec4f laid out as [x, y, z, w] (ROS convention)
        return { qx, qy, qz, qw };
    }

    inline cv::Point3f positionFromMatx44f(const cv::Matx44f& T) 
    {
        return { T(0,3), T(1,3), T(2,3) };
    }

} // namespace Utils

#endif