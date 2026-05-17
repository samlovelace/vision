#ifndef MIDASMODEL_H
#define MIDASMODEL_H
 
#include "IModel.hpp"
#include <opencv2/opencv.hpp>
 
class MidasModel : public IModel
{ 
public:
    MidasModel();
    ~MidasModel();

    bool init() override; 
    bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) override; 
    bool postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output) override; 

private:
   
};
#endif //MIDASMODEL_H