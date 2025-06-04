#include "DepthEstimatorFactory.h"
#include "MonocularDepthEstimator.h"
#include "plog/Log.h"


// std::shared_ptr<IDepthEstimator> DepthEstimatorFactory::create(const std::string& aType)
// {
//     if("monocular" == aType)
//     {
//         return std::make_shared<MonocularDepthEstimator>(); 
//     }
//     else
//     {
//         LOGE << "Unsupported depth estimator of type: " << aType; 
//         return nullptr; 
//     }
// }