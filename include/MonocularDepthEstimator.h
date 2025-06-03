#ifndef MONOCULARDEPTHESTIMATOR_H
#define MONOCULARDEPTHESTIMATOR_H
 
#include "IDepthEstimator.hpp"
 
class MonocularDepthEstimator : public IDepthEstimator
{ 
public:
    MonocularDepthEstimator();
    ~MonocularDepthEstimator();

    bool estimateDepth() override; 

private:
   
};
#endif //MONOCULARDEPTHESTIMATOR_H