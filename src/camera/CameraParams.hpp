#ifndef CAMERAPARAMS_HPP
#define CAMERAPARAMS_HPP

#include <opencv2/core/matx.hpp>
#include <memory>

struct CameraIntrinsics
{
    CameraIntrinsics(float fx, float fy, float cx, float cy, float near, float far)
            : intrinsicMatrix(fx, 0, cx, 0, fy, cy, 0, 0, 1),
            nearPlane_m(near), farPlane_m(far) {}

    CameraIntrinsics(std::vector<float> focal, std::vector<float> center, float near, float far) 
        : intrinsicMatrix(focal[0], 0, center[0], 0, focal[1], center[1], 0, 0, 1), 
        nearPlane_m(near), farPlane_m(far) {}

    cv::Matx33f intrinsicMatrix;
    float nearPlane_m;
    float farPlane_m;

    float focalX() const { return intrinsicMatrix(0, 0); }
    float focalY() const { return intrinsicMatrix(1, 1); }
    float centerX() const { return intrinsicMatrix(0, 2); }
    float centerY() const { return intrinsicMatrix(1, 2); }
};

struct CameraParams
{
    CameraParams(std::shared_ptr<CameraIntrinsics> anIntrinsics, cv::Matx44f anS2V) : mIntrinsics(anIntrinsics), mS2V(anS2V) {}

    std::shared_ptr<CameraIntrinsics> mIntrinsics; 
    cv::Matx44f mS2V; 
};



#endif