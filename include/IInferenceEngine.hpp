#ifndef IINFERENCEENGINE_H
#define IINFERENCEENGINE_H

#include <opencv4/opencv2/core/core.hpp>
 
class IInferenceEngine 
{ 
public:
    ~IInferenceEngine() = default; 

    virtual bool init() = 0; 
    virtual bool loadModel(const std::string& aModelPath) = 0; 
    virtual void doInference(const cv::Mat& anImage) = 0; 

private:
   
};
#endif //IINFERENCEENGINE_H