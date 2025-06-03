#ifndef IDEPTHESTIMATOR_HPP
#define IDEPTHESTIMATOR_HPP
 
 
class IDepthEstimator 
{ 
public:
    ~IDepthEstimator() = default; 

    virtual bool estimateDepth() = 0; 

private:
   
};
#endif //IDEPTHESTIMATOR_HPP