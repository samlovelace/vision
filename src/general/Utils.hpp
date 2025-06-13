#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>

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


} // namespace Utils

#endif