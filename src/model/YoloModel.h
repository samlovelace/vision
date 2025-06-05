#ifndef YOLOMODEL_H
#define YOLOMODEL_H
 
#include "IModel.hpp"
 
class YoloModel : public IModel
{ 
public:
    YoloModel();
    ~YoloModel();

    bool init() override; 
    bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) override; 
    bool postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output) override; 

private:
   
};
#endif //YOLOMODEL_H