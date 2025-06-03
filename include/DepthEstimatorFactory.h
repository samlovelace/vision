#ifndef DEPTHESTIMATORFACTORY_H
#define DEPTHESTIMATORFACTORY_H
 
#include "IDepthEstimator.hpp"; 
#include <memory>
 
class DepthEstimatorFactory 
{ 
public:
    DepthEstimatorFactory() {}
    ~DepthEstimatorFactory() {}

    static std::shared_ptr<IDepthEstimator> create(const std::string& aType); 

private:
   
};
#endif //DEPTHESTIMATORFACTORY_H