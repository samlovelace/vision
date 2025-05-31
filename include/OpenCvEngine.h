#ifndef OPENCVENGINE_H
#define OPENCVENGINE_H
 
#include "IInferenceEngine.hpp"
 
class OpenCvEngine : public IInferenceEngine
{ 
public:
    OpenCvEngine();
    ~OpenCvEngine();

    bool init() override; 
    bool loadModel(const std::string& aModelPath) override; 
    void doInference(const cv::Mat& anImage) override; 

private:
   
};
#endif //OPENCVENGINE_H