#ifndef OPENCVENGINE_H
#define OPENCVENGINE_H
 
#include "IInferenceEngine.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
 
class OpenCvEngine : public IInferenceEngine
{ 
public:
    OpenCvEngine();
    ~OpenCvEngine();

    bool init() override; 
    bool loadModel(const std::string& aModelPath) override; 
    cv::Mat doInference(const cv::Mat& anImage) override; 

private:
    cv::dnn::Net mNet; 
   
};
#endif //OPENCVENGINE_H