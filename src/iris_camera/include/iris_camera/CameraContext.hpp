#ifndef CAMERACONTEXT_HPP
#define CAMERACONTEXT_HPP

#include <memory>

#include "iris_camera/ICamera.hpp"
#include "iris_common/ConcurrentQueue.hpp"
#include "iris_camera/CameraParams.hpp"
#include "iris_camera/RateController.h"

struct CameraContext
{
    CameraContext(std::shared_ptr<RateController> aRate, 
                  std::shared_ptr<ICamera> aCamera, 
                  std::shared_ptr<CameraParams> aCamParams) : 
        mRate(aRate), mCamera(aCamera), mParams(aCamParams) {}

    std::shared_ptr<RateController> mRate; 
    std::shared_ptr<ICamera> mCamera; 
    std::shared_ptr<CameraParams> mParams; 
};

#endif
